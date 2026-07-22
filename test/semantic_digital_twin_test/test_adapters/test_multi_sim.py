import logging
import os
import threading
import time

import mujoco
import pytest
import numpy
from PIL import Image
from scipy.spatial.transform import Rotation

from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.exceptions import (
    ParsingError,
    UnsupportedConnection6DoFParentError,
)
from semantic_digital_twin.robots.hsrb import HSRB
from semantic_digital_twin.robots.tracy import Tracy
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
    OmniDrive,
    RevoluteConnection,
)
from semantic_digital_twin.world_description.degree_of_freedom import DegreeOfFreedom
from semantic_digital_twin.world_description.geometry import (
    Box,
    Scale,
    Color,
    Cylinder,
    Mesh,
    Texture,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body, Region, Actuator

from physics_simulators.mujoco_simulator import MujocoSimulator
from physics_simulators.base_simulator import SimulatorState
from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.adapters.multi_sim import (
    MujocoSim,
    MujocoActuator,
    MujocoBuilder,
    MujocoLight,
    MujocoSynchronizer,
)

urdf_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "..",
    "..",
    "semantic_digital_twin",
    "resources",
    "urdf",
)
mjcf_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "..",
    "..",
    "semantic_digital_twin",
    "resources",
    "mjcf",
)

logger = logging.getLogger(__name__)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter("[%(levelname)s] %(message)s"))
if not logger.handlers:
    logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

headless = os.environ.get("CI", "false").lower() == "true"
only_run_test_in_CI = os.environ.get("CI", "false").lower() == "false"

pytestmark = pytest.mark.skipif(
    only_run_test_in_CI,
    reason="Only run test in CI or multisim could not be imported.",
)

TEST_URDF_1 = os.path.normpath(os.path.join(urdf_dir, "simple_two_arm_robot.urdf"))
TEST_URDF_2 = HSRB.get_ros_file_path()
TEST_URDF_TRACY = Tracy.get_ros_file_path()
TEST_MJCF_1 = os.path.normpath(os.path.join(mjcf_dir, "mjx_single_cube_no_mesh.xml"))
TEST_MJCF_2 = os.path.normpath(os.path.join(mjcf_dir, "jeroen_cups.xml"))
STEP_SIZE = 1e-3


def stop_multisim_if_running(multi_sim: MujocoSim) -> None:
    simulator = getattr(multi_sim, "simulator", None)
    if simulator is None:
        return
    if getattr(simulator, "state", None) is SimulatorState.STOPPED:
        return
    multi_sim.stop_simulation()


def _spawn_revolute_joint(
    world: World, joint_name: str, parent: Body = None
) -> DegreeOfFreedom:
    """
    Adds a single-dof revolute joint, connected to ``parent`` (``world.root`` by
    default), and returns its :class:`DegreeOfFreedom`. ``multi_sim`` must already be
    built so ``world.root`` exists and the connection gets spawned into MuJoCo live.
    """
    body = Body(name=PrefixedName(f"{joint_name}_body"))
    dof = DegreeOfFreedom(name=PrefixedName(joint_name))
    with world.modify_world():
        world.add_degree_of_freedom(dof)
        world.add_connection(
            RevoluteConnection(
                name=dof.name,
                parent=parent or world.root,
                child=body,
                axis=Vector3.Z(reference_frame=body),
                raw_dof=dof,
            )
        )
    return dof


def _qpos_adr(multi_sim: MujocoSim, joint_name: str) -> int:
    mj_model = multi_sim.simulator._mj_model
    joint_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    assert joint_id != -1, f"joint {joint_name} not found in the compiled MuJoCo model"
    return mj_model.jnt_qposadr[joint_id]


class _RecordingLogHandler(logging.Handler):
    """
    Collects log records emitted by a specific named logger.

    ``caplog`` relies on records propagating up to the root logger, but the
    ``semantic_digital_twin``/``semantic_digital_twin.adapters.multi_sim`` loggers have
    ``propagate=False`` in this environment (likely set up by the ROS/ament logging
    integration pulled in transitively), so records never reach caplog's root-attached
    handler. Attaching a handler directly to the target logger sidesteps that.
    """

    def __init__(self, logger_name: str, level=logging.WARNING):
        super().__init__(level=level)
        self.records: list = []
        self._logger = logging.getLogger(logger_name)

    def emit(self, record: logging.LogRecord) -> None:
        self.records.append(record)

    def __enter__(self) -> "_RecordingLogHandler":
        self._logger.addHandler(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self._logger.removeHandler(self)

    def has_message_containing(self, text: str) -> bool:
        return any(text in record.getMessage() for record in self.records)


@pytest.fixture
def test_urdf_1_world():
    return URDFParser.from_file(file_path=TEST_URDF_1).parse()


@pytest.fixture
def test_mjcf_1_world():
    return MJCFParser(TEST_MJCF_1).parse()


@pytest.fixture
def test_mjcf_2_world():
    return MJCFParser(TEST_MJCF_2).parse()


def test_empty_multi_sim_in_5s():
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_world_multi_sim_in_5s(test_urdf_1_world):
    multi_sim = MujocoSim(world=test_urdf_1_world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_apartment_multi_sim_in_5s():
    try:
        test_urdf_2_world = URDFParser.from_file(file_path=TEST_URDF_2).parse()
    except ParsingError:
        pytest.skip("Skipping HSRB krrood_test due to URDF parsing error.")

    multi_sim = MujocoSim(world=test_urdf_2_world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_world_multi_sim_with_change(test_urdf_1_world):
    multi_sim = MujocoSim(world=test_urdf_1_world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        time.sleep(1.0)

        start_time = time.time()

        new_body = Body(name=PrefixedName("test_body"))
        box_origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0.2, y=0.4, z=3.0, roll=0, pitch=0.5, yaw=0, reference_frame=new_body
        )
        box = Box(
            origin=box_origin,
            scale=Scale(1.0, 1.5, 0.5),
            color=Color(1.0, 0.0, 0.0, 1.0),
        )
        new_body.collision = ShapeCollection([box], reference_frame=new_body)

        logger.debug(f"Time before adding new body: {time.time() - start_time}s")
        with test_urdf_1_world.modify_world():
            test_urdf_1_world.add_connection(
                Connection6DoF.create_with_dofs(
                    world=test_urdf_1_world,
                    parent=test_urdf_1_world.root,
                    child=new_body,
                )
            )
        logger.debug(f"Time after adding new body: {time.time() - start_time}s")

        assert new_body.name.name in multi_sim.simulator.get_all_body_names().result

        time.sleep(0.5)

        region = Region(name=PrefixedName("test_region"))
        region_box = Box(
            scale=Scale(0.1, 0.5, 0.2),
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(reference_frame=region),
            color=Color(0.0, 1.0, 0.0, 0.8),
        )
        region.area = ShapeCollection([region_box], reference_frame=region)

        logger.debug(f"Time before add adding region: {time.time() - start_time}s")
        with test_urdf_1_world.modify_world():
            test_urdf_1_world.add_connection(
                FixedConnection(
                    parent=test_urdf_1_world.root,
                    child=region,
                    parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                        z=0.5
                    ),
                )
            )
        logger.debug(f"Time after add adding region: {time.time() - start_time}s")

        assert region.name.name in multi_sim.simulator.get_all_body_names().result

        time.sleep(0.5)

        T_const = 0.1
        kp = 100
        kv = 10
        actuator = Actuator()
        dof = test_urdf_1_world.get_degree_of_freedom_by_name(name="r_joint_1")
        actuator.add_dof(dof=dof)
        actuator.simulator_additional_properties.append(
            MujocoActuator(
                dynamics_type=mujoco.mjtDyn.mjDYN_NONE,
                dynamics_parameters=[T_const] + [0.0] * 9,
                gain_type=mujoco.mjtGain.mjGAIN_FIXED,
                gain_parameters=[kp] + [0.0] * 9,
                bias_type=mujoco.mjtBias.mjBIAS_AFFINE,
                bias_parameters=[0, -kp, -kv] + [0.0] * 7,
            )
        )

        logger.debug(f"Time before adding new actuator: {time.time() - start_time}s")
        with test_urdf_1_world.modify_world():
            test_urdf_1_world.add_actuator(actuator=actuator)
        logger.debug(f"Time after adding new actuator: {time.time() - start_time}s")

        assert actuator.name.name in multi_sim.simulator.get_all_actuator_names().result

        time.sleep(4.0)
        multi_sim.stop_simulation()
    finally:
        stop_multisim_if_running(multi_sim)


def test_multi_sim_in_5s(test_mjcf_1_world):
    multi_sim = MujocoSim(
        world=test_mjcf_1_world,
        headless=headless,
        step_size=STEP_SIZE,
    )

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_mesh_scale_and_equality(test_mjcf_2_world):
    multi_sim = MujocoSim(
        world=test_mjcf_2_world,
        headless=headless,
        step_size=STEP_SIZE,
    )

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def _write_textured_tetrahedron(directory, texture_color) -> str:
    """
    Writes a minimal textured OBJ+MTL+PNG mesh (a tetrahedron, so its convex hull is
    non-degenerate) into ``directory``, textured with a solid ``texture_color``, and returns
    the OBJ file's path. Always named "tetra.obj"/"tetra.mtl"/"wood.png", so callers writing
    into different directories can reproduce a texture basename collision between them.
    """
    directory.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", (4, 4), color=texture_color).save(directory / "wood.png")
    (directory / "tetra.mtl").write_text("newmtl wood\nmap_Kd wood.png\n")
    mesh_file = directory / "tetra.obj"
    mesh_file.write_text(
        "mtllib tetra.mtl\n"
        "o tetra\n"
        "v 0.0 0.0 0.0\n"
        "v 1.0 0.0 0.0\n"
        "v 0.0 1.0 0.0\n"
        "v 0.0 0.0 1.0\n"
        "vt 0.0 0.0\n"
        "vt 1.0 0.0\n"
        "vt 0.0 1.0\n"
        "vt 0.5 0.5\n"
        "usemtl wood\n"
        "f 1/1 2/2 3/3\n"
        "f 1/1 2/2 4/4\n"
        "f 1/1 3/3 4/4\n"
        "f 2/2 3/3 4/4\n"
    )
    return str(mesh_file)


def _build_world_with_two_textured_bodies(
    tmp_path, mesh_file_a: str, mesh_file_b: str
) -> MujocoBuilder:
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        for name, mesh_file in [("quad_0", mesh_file_a), ("quad_1", mesh_file_b)]:
            mesh_shape = Mesh(filename=mesh_file, scale=Scale(1, 1, 1))
            body = Body(
                name=PrefixedName(name),
                visual=ShapeCollection([mesh_shape]),
                collision=ShapeCollection([mesh_shape]),
            )
            world.add_kinematic_structure_entity(body)
            world.add_connection(FixedConnection(parent=root, child=body))

    builder = MujocoBuilder()
    builder.build_world(world=world, file_path=str(tmp_path / "scene.xml"))
    return builder


def test_builder_assigns_material_to_every_geom_sharing_a_texture(tmp_path):
    """
    Regression test: MujocoBuilder._parse_geom used to return early - without ever setting
    geom_props["material"] - whenever a geom's texture was already registered by an earlier
    geom. Since most textures in a scene are shared across many geoms (a real RoboCasa
    kitchen reuses a handful of textures across ~90 meshes), this meant only the first geom
    to use a given texture ever got a material; every later reuse silently rendered with
    MuJoCo's default (untextured, gray) material instead.
    """
    mesh_file = _write_textured_tetrahedron(tmp_path, texture_color=(120, 60, 20))

    builder = _build_world_with_two_textured_bodies(tmp_path, mesh_file, mesh_file)

    materials = {
        body.name: geom.material for body in builder.spec.bodies for geom in body.geoms
    }
    assert materials["quad_0"] == materials["quad_1"]
    assert materials["quad_0"] != ""


def test_builder_does_not_confuse_different_textures_sharing_a_basename(tmp_path):
    """
    Regression test: RoboCasa's asset pipeline reuses generic texture basenames (e.g.
    "T_BC001.png") across many unrelated fixtures' own distinct texture files - a real
    kitchen had 14 different fixtures (sink, stove, fridge, dishwasher, ...) all using a
    texture file named exactly "T_BC001.png" in their own directories. Deduplicating by
    basename alone collapsed all of them onto whichever fixture's texture was registered
    first, so most fixtures rendered with the wrong (borrowed) texture image instead of
    their own.
    """
    mesh_file_a = _write_textured_tetrahedron(
        tmp_path / "fixture_a", texture_color=(200, 0, 0)
    )
    mesh_file_b = _write_textured_tetrahedron(
        tmp_path / "fixture_b", texture_color=(0, 200, 0)
    )

    builder = _build_world_with_two_textured_bodies(tmp_path, mesh_file_a, mesh_file_b)

    materials = {
        body.name: geom.material for body in builder.spec.bodies for geom in body.geoms
    }
    assert materials["quad_0"] != materials["quad_1"]
    texture_files = {texture.name: texture.file for texture in builder.spec.textures}
    assert len(texture_files) == 2


def test_builder_writes_a_light_attached_to_a_body(tmp_path):
    """
    Regression test: MujocoBuilder had no handling for MujocoLight additional properties at
    all, so a world's lights were silently dropped when built into a MuJoCo scene - every
    recorded/simulated world fell back to MuJoCo's minimal default camera headlight instead
    of the scene's own intended lighting.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        root.simulator_additional_properties.append(
            MujocoLight(
                name="overview_light",
                body=root,
                directional=True,
                position=[2.0, -2.0, 2.0],
                direction=[0.0, 0.0, -1.0],
                ambient=[0.3, 0.3, 0.3],
                diffuse=[0.5, 0.5, 0.5],
                specular=[0.3, 0.3, 0.3],
            )
        )

    builder = MujocoBuilder()
    builder.build_world(world=world, file_path=str(tmp_path / "scene.xml"))

    [light] = [light for body in builder.spec.bodies for light in body.lights]
    assert light.name == "overview_light"
    assert list(light.pos) == pytest.approx([2.0, -2.0, 2.0])
    assert list(light.ambient) == pytest.approx([0.3, 0.3, 0.3])
    assert list(light.diffuse) == pytest.approx([0.5, 0.5, 0.5])


def test_builder_assigns_material_to_a_textured_primitive_shape(tmp_path):
    """
    Regression test: Box/Sphere/Cylinder shapes never carried any texture reference, only a
    flat Color - RoboCasa's countertops and cabinet doors are actual MJCF box geoms with a
    material referencing a marble/wood texture, so this whole texture reference was silently
    discarded on every round-trip and they rendered flat-colored instead of textured.
    """
    texture_directory = tmp_path / "textures"
    texture_directory.mkdir()
    texture_file = texture_directory / "marble.png"
    Image.new("RGB", (4, 4), color=(200, 200, 200)).save(texture_file)

    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("root"))
        world.add_body(root)
        box_shape = Box(
            scale=Scale(1, 1, 1),
            texture=Texture(
                file_path=str(texture_file), repeat=(3.0, 3.0), uniform=True
            ),
        )
        counter = Body(
            name=PrefixedName("counter"),
            visual=ShapeCollection([box_shape]),
            collision=ShapeCollection([box_shape]),
        )
        world.add_kinematic_structure_entity(counter)
        world.add_connection(FixedConnection(parent=root, child=counter))

    builder = MujocoBuilder()
    builder.build_world(world=world, file_path=str(tmp_path / "scene.xml"))

    [geom] = [
        geom
        for body in builder.spec.bodies
        for geom in body.geoms
        if body.name == "counter"
    ]
    assert geom.material != ""
    [material] = [
        material
        for material in builder.spec.materials
        if material.name == geom.material
    ]
    assert list(material.texrepeat) == pytest.approx([3.0, 3.0])
    assert bool(material.texuniform) is True
    texture_name = material.textures[0]
    assert texture_name != ""
    [texture] = [
        texture for texture in builder.spec.textures if texture.name == texture_name
    ]
    assert texture.file == str(texture_file)


def test_mujoco_with_tracy_dae_files():
    try:
        dae_world = URDFParser.from_file(file_path=TEST_URDF_TRACY).parse()
    except ParsingError:
        pytest.skip("Skipping tracy test due to URDF parsing error.")

    multi_sim = MujocoSim(world=dae_world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_mujocosim_world_with_added_objects(test_urdf_1_world):
    milk_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "..",
        "..",
        "semantic_digital_twin",
        "resources",
        "stl",
        "milk.stl",
    )
    stl_parser = STLParser(milk_path)
    mesh_world = stl_parser.parse()
    transformation = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=0.5, reference_frame=test_urdf_1_world.root
    )

    with test_urdf_1_world.modify_world():
        test_urdf_1_world.merge_world_at_pose(mesh_world, transformation)

    multi_sim = MujocoSim(world=test_urdf_1_world, headless=headless)

    try:
        assert isinstance(multi_sim.simulator, MujocoSimulator)
        assert multi_sim.simulator.file_path == MujocoSim.default_file_path
        assert multi_sim.simulator.headless is headless
        assert multi_sim.simulator.step_size == STEP_SIZE

        multi_sim.start_simulation()
        start_time = time.time()
        time.sleep(5.0)
        multi_sim.stop_simulation()

        assert time.time() - start_time >= 5.0
    finally:
        stop_multisim_if_running(multi_sim)


def test_spawn_body_with_connections():
    def spawn_robot_body(spawn_world: World) -> Body:
        spawn_body = Body(name=PrefixedName("robot"))
        box_origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=0, y=0, z=0.5, roll=0, pitch=0, yaw=0, reference_frame=spawn_body
        )
        box = Box(
            origin=box_origin,
            scale=Scale(0.4, 0.4, 1.0),
            color=Color(0.9, 0.9, 0.9, 1.0),
        )
        spawn_body.collision = ShapeCollection([box], reference_frame=spawn_body)

        with spawn_world.modify_world():
            spawn_world.add_connection(
                FixedConnection(parent=spawn_world.root, child=spawn_body)
            )

        return spawn_body

    def spawn_shoulder_bodies(spawn_world: World, root_body: Body) -> tuple[Body, Body]:
        spawn_left_shoulder_body = Body(name=PrefixedName("left_shoulder"))
        cylinder = Cylinder(
            width=0.2,
            height=0.1,
            color=Color(0.9, 0.1, 0.1, 1.0),
        )
        spawn_left_shoulder_body.collision = ShapeCollection(
            [cylinder], reference_frame=spawn_left_shoulder_body
        )
        dof = DegreeOfFreedom(name=PrefixedName("left_shoulder_joint"))
        left_shoulder_origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=0,
            pos_y=0.3,
            pos_z=0.9,
            quat_w=0.707,
            quat_x=0.707,
            quat_y=0,
            quat_z=0,
        )

        with spawn_world.modify_world():
            spawn_world.add_degree_of_freedom(dof)
            spawn_world.add_connection(
                RevoluteConnection(
                    name=dof.name,
                    parent=root_body,
                    child=spawn_left_shoulder_body,
                    axis=Vector3.Z(reference_frame=spawn_left_shoulder_body),
                    raw_dof=dof,
                    parent_T_connection_expression=left_shoulder_origin,
                )
            )

        spawn_right_shoulder_body = Body(name=PrefixedName("right_shoulder"))
        cylinder = Cylinder(
            width=0.2,
            height=0.1,
            color=Color(0.9, 0.1, 0.1, 1.0),
        )
        spawn_right_shoulder_body.collision = ShapeCollection(
            [cylinder], reference_frame=spawn_right_shoulder_body
        )
        dof = DegreeOfFreedom(name=PrefixedName("right_shoulder_joint"))
        right_shoulder_origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
            pos_x=0,
            pos_y=-0.3,
            pos_z=0.9,
            quat_w=0.707,
            quat_x=0.707,
            quat_y=0,
            quat_z=0,
        )

        with spawn_world.modify_world():
            spawn_world.add_degree_of_freedom(dof)
            spawn_world.add_connection(
                RevoluteConnection(
                    name=dof.name,
                    parent=root_body,
                    child=spawn_right_shoulder_body,
                    axis=Vector3.Z(reference_frame=spawn_right_shoulder_body),
                    raw_dof=dof,
                    parent_T_connection_expression=right_shoulder_origin,
                )
            )

        return spawn_left_shoulder_body, spawn_right_shoulder_body

    world = World()
    multi_sim = MujocoSim(
        world=world,
        headless=headless,
        step_size=0.001,
    )

    try:
        multi_sim.start_simulation()
        time.sleep(1)

        robot_body = spawn_robot_body(spawn_world=world)
        spawn_shoulder_bodies(spawn_world=world, root_body=robot_body)

        time.sleep(1)

        assert set(multi_sim.simulator.get_all_body_names().result) == {
            "world",
            "robot",
            "left_shoulder",
            "right_shoulder",
        }

        multi_sim.stop_simulation()
    finally:
        stop_multisim_if_running(multi_sim)


def test_body_frame_excludes_joint_state_at_build_time():
    """A body's static frame must be built at the reference (zero-joint) pose.

    The joint is non-zero while the simulator is built and is evaluated at a
    different angle, so a frame that baked in the build-time angle would have it
    applied twice and drift away from the world forward kinematics.
    """
    world = World()
    base_body = Body(name=PrefixedName("base"))
    rotated_link = Body(name=PrefixedName("rotated_link"))
    # A tip offset from the joint axis, so a rotation actually moves its position
    # (the joint child sits on the axis and would not reveal the bug).
    tip_link = Body(name=PrefixedName("tip"))
    rotated_origin = HomogeneousTransformationMatrix.from_xyz_quaternion(
        pos_x=0.3,
        pos_y=0.0,
        pos_z=0.9,
        quat_w=0.707,
        quat_x=0.707,
        quat_y=0.0,
        quat_z=0.0,
    )
    tip_offset = HomogeneousTransformationMatrix.from_xyz_rpy(x=0.5, y=0.2, z=0.0)
    rotated_joint_dof = DegreeOfFreedom(name=PrefixedName("rotated_joint"))
    with world.modify_world():
        world.add_body(base_body)
        world.add_degree_of_freedom(rotated_joint_dof)
        world.add_connection(
            RevoluteConnection(
                name=rotated_joint_dof.name,
                parent=base_body,
                child=rotated_link,
                axis=Vector3.Z(reference_frame=rotated_link),
                raw_dof=rotated_joint_dof,
                parent_T_connection_expression=rotated_origin,
            )
        )
        world.add_connection(
            FixedConnection(
                parent=rotated_link,
                child=tip_link,
                parent_T_connection_expression=tip_offset,
            )
        )

    build_time_angle = 0.7
    with world.modify_world():
        world.state[rotated_joint_dof.id].position = build_time_angle

    multi_sim = MujocoSim(world=world, headless=headless, step_size=0.001)
    try:
        evaluation_angle = 0.3
        with world.modify_world():
            world.state[rotated_joint_dof.id].position = evaluation_angle

        mujoco_model = multi_sim.simulator._mj_model
        mujoco_data = multi_sim.simulator._mj_data
        joint_id = mujoco.mj_name2id(
            mujoco_model, mujoco.mjtObj.mjOBJ_JOINT, rotated_joint_dof.name.name
        )
        mujoco_data.qpos[mujoco_model.jnt_qposadr[joint_id]] = evaluation_angle
        mujoco.mj_forward(mujoco_model, mujoco_data)

        simulated_position = multi_sim.simulator.get_body_position(
            tip_link.name.name
        ).result[:3]
        world_position = world.compute_forward_kinematics_np(world.root, tip_link)[
            :3, 3
        ]
        numpy.testing.assert_allclose(simulated_position, world_position, atol=1e-4)
    finally:
        stop_multisim_if_running(multi_sim)


def test_omni_drive_spawn_pose_is_baked_into_static_body_frame(tmp_path):
    """
    Regression test: OmniDrive (and DifferentialDrive) never get a MuJoCo joint built for
    them (see MultiSimBuilder._ignore_connection_types), so nothing else carries their
    live x/y/yaw state into the exported scene. KinematicStructureEntityConverter._convert
    used to always read reference_origin_as_position_quaternion(), which excludes that
    live state - a robot spawned at a non-identity OmniDrive pose therefore ended up at
    the world origin in MuJoCo, while e.g. RViz (which reads the full
    origin_as_position_quaternion()) showed it at the correct spawn pose.
    """
    world = World()
    with world.modify_world():
        map_root = Body(name=PrefixedName("map"))
        world.add_body(map_root)
        robot_root = Body(name=PrefixedName("robot_root"))
        drive = OmniDrive.create_with_dofs(
            world=world, parent=map_root, child=robot_root
        )
        world.add_connection(drive)

    drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
        x=1.5, y=-0.5, yaw=0.9, reference_frame=map_root
    )

    builder = MujocoBuilder()
    builder.build_world(world=world, file_path=str(tmp_path / "scene.xml"))

    [robot_body] = [
        body for body in builder.spec.bodies if body.name == "robot_root"
    ]

    expected_pose = world.compute_forward_kinematics_np(world.root, robot_root)
    expected_position = expected_pose[:3, 3]
    expected_quat_wxyz = Rotation.from_matrix(expected_pose[:3, :3]).as_quat(
        scalar_first=True
    )

    assert not numpy.allclose(expected_position, [0.0, 0.0, 0.0])
    numpy.testing.assert_allclose(
        list(robot_body.pos), expected_position, atol=1e-6
    )
    numpy.testing.assert_allclose(
        list(robot_body.quat), expected_quat_wxyz, atol=1e-6
    )


def test_world_sim_state_sync():
    plane_half_thickness = 0.05
    box_half_size = 0.1
    init_pos = numpy.array([0.3, 0.2, 5.0])
    target_pos = numpy.array(
        [init_pos[0], init_pos[1], plane_half_thickness + box_half_size]
    )

    def spawn_state_sync_scene(
        spawn_world: World,
    ) -> tuple[Body, Connection6DoF]:
        plane_body = Body(name=PrefixedName("ground_plane"))
        plane_body.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        reference_frame=plane_body
                    ),
                    scale=Scale(2.0, 2.0, plane_half_thickness * 2),
                    color=Color(1.0, 1.0, 0.0, 1.0),
                )
            ],
            reference_frame=plane_body,
        )

        falling_box = Body(name=PrefixedName("falling_box"))
        falling_box.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                        reference_frame=falling_box
                    ),
                    scale=Scale(
                        box_half_size * 2, box_half_size * 2, box_half_size * 2
                    ),
                    color=Color(1.0, 0.0, 0.0, 1.0),
                )
            ],
            reference_frame=falling_box,
        )

        with spawn_world.modify_world():
            spawn_world.add_connection(
                FixedConnection(parent=spawn_world.root, child=plane_body)
            )
            box_connection = Connection6DoF.create_with_dofs(
                world=spawn_world,
                parent=spawn_world.root,
                child=falling_box,
            )
            spawn_world.add_connection(box_connection)
        return falling_box, box_connection

    world = World()
    multi_sim = MujocoSim(
        world=world,
        headless=headless,
        step_size=STEP_SIZE,
    )

    try:
        multi_sim.start_simulation()
        time.sleep(1)

        falling_box, box_connection = spawn_state_sync_scene(world)

        body_names = multi_sim.simulator.get_all_body_names().result
        assert {"ground_plane", "falling_box"}.issubset(
            body_names
        ), f"scene bodies were not spawned in the simulator; bodies={body_names}"

        box_connection.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
            x=float(init_pos[0]),
            y=float(init_pos[1]),
            z=float(init_pos[2]),
            reference_frame=falling_box,
        )
        time.sleep(2.5)

        final_pos = numpy.asarray(
            multi_sim.simulator.get_body_position("falling_box").result[:3],
            dtype=float,
        )

        multi_sim.stop_simulation()

        assert numpy.allclose(final_pos, target_pos, atol=1e-1), (
            f"Box did not settle at target: final_pos={final_pos}, "
            f"expected≈{target_pos}"
        )
    finally:
        stop_multisim_if_running(multi_sim)


def test_reset_simulation_resyncs_world_state():
    """
    Regression test: MultiSim.reset_simulation used to only call simulator.reset()
    without pulling the reset state back into world.state. The sim -> world direction
    is otherwise driven only by the physics step loop, which a reset does not go
    through, so world.state kept showing the pre-reset joint value until (if ever) the
    simulation was stepped again.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        dof = _spawn_revolute_joint(world, "reset_test_joint")

        world.state[dof.id].position = 1.2
        world.notify_state_change()
        assert multi_sim.simulator._mj_data.qpos[
            _qpos_adr(multi_sim, "reset_test_joint")
        ] == pytest.approx(1.2)

        multi_sim.reset_simulation()

        assert world.state[dof.id].position == pytest.approx(0.0)
    finally:
        stop_multisim_if_running(multi_sim)


def test_sim_to_world_sync_does_not_drop_concurrent_edits_during_pause_window():
    """
    Regression test: MujocoSynchronizer._sim_to_world pauses its sibling state-change
    callback, pulls qpos into world.state, then used to rebase the *entire*
    previous-state snapshot to whatever world.state held at that moment - via
    StateChangeCallback.update_previous_world_state(), a full positional copy of
    world.state.positions - before resuming. A dof that a different thread edited
    concurrently, while the callback happened to be paused, was swept up in that same
    full rebase and looked "already synced", so the edit was silently never pushed to
    MuJoCo and never retried.

    joint_a is a real MuJoCo joint that _sim_to_world actually reads on every call, so
    it always triggers the "something changed" rebase path. joint_b's own resolution
    is monkeypatched away for the duration of the _sim_to_world call (simulating, e.g.,
    a moment where a spawn is still in flight) so the loop never touches it, isolating
    the edit made to it during the pause window from being simply overwritten by the
    sim's own read of the same dof.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        _spawn_revolute_joint(world, "pause_window_joint_a")
        dof_b = _spawn_revolute_joint(world, "pause_window_joint_b")
        connection_b = next(
            c for c in world.connections if getattr(c, "raw_dof", None) is dof_b
        )

        original_resolve_qpos_adr = multi_sim.synchronizer._resolve_qpos_adr
        skip_joint_b = {"active": True}

        def resolve_qpos_adr_hiding_b(connection):
            if skip_joint_b["active"] and connection is connection_b:
                return None
            return original_resolve_qpos_adr(connection)

        multi_sim.synchronizer._resolve_qpos_adr = resolve_qpos_adr_hiding_b

        original_pause = multi_sim.synchronizer._state_callback.pause

        def pause_and_make_concurrent_edit():
            original_pause()
            # simulates a different thread mutating world.state for joint_b while our
            # callback is paused, without going through notify_state_change (which
            # would be a no-op while paused anyway).
            world.state[dof_b.id].position = 0.42

        multi_sim.synchronizer._state_callback.pause = pause_and_make_concurrent_edit

        multi_sim.synchronizer.sync_rate_hz = MujocoSynchronizer.UNTHROTTLED_SYNC_RATE_HZ
        multi_sim.synchronizer._sim_to_world()

        # joint_b is resolvable again, matching the spawn having finished; the edit
        # made during the pause window must still be pending.
        skip_joint_b["active"] = False
        world.notify_state_change()

        assert multi_sim.simulator._mj_data.qpos[
            _qpos_adr(multi_sim, "pause_window_joint_b")
        ] == pytest.approx(0.42)
    finally:
        stop_multisim_if_running(multi_sim)


def test_world_to_sim_sync_holds_model_lock():
    """
    Regression test: MujocoSynchronizer._on_state_change used to write directly into
    _mj_data.qpos without acquiring the simulator's _model_lock - the very lock
    step_callback holds while running mj_step - so a state push from a user thread
    could race a concurrently running physics step over the same qpos array. This
    proves _on_state_change now blocks on that lock instead of writing straight
    through it.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        dof = _spawn_revolute_joint(world, "lock_test_joint")

        lock_acquired_by_holder = threading.Event()
        release_lock = threading.Event()

        def hold_lock():
            with multi_sim.simulator._model_lock:
                lock_acquired_by_holder.set()
                release_lock.wait(timeout=5.0)

        holder_thread = threading.Thread(target=hold_lock)
        holder_thread.start()
        assert lock_acquired_by_holder.wait(timeout=2.0)

        state_change_finished = threading.Event()

        def push_state_change():
            world.state[dof.id].position = 0.9
            world.notify_state_change()
            state_change_finished.set()

        pusher_thread = threading.Thread(target=push_state_change)
        pusher_thread.start()

        # the pusher should be blocked on _model_lock as long as hold_lock holds it
        assert not state_change_finished.wait(timeout=0.5)

        release_lock.set()
        holder_thread.join(timeout=2.0)
        assert state_change_finished.wait(timeout=2.0)
        pusher_thread.join(timeout=2.0)
    finally:
        stop_multisim_if_running(multi_sim)


def test_sim_to_world_sync_does_not_notify_when_nothing_moved():
    """
    Regression test: MujocoSynchronizer._sim_to_world used to call
    world.notify_state_change() - bumping world.state.version and firing every
    registered state-change callback - on every throttled tick as long as at least one
    connection resolved to a MuJoCo joint, regardless of whether any value actually
    changed. On a resting scene this fired a spurious notification on every tick.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        _spawn_revolute_joint(world, "resting_joint")

        multi_sim.synchronizer.sync_rate_hz = MujocoSynchronizer.UNTHROTTLED_SYNC_RATE_HZ
        multi_sim.synchronizer._sim_to_world()  # establish the baseline snapshot

        version_before = world.state.version
        multi_sim.synchronizer._sim_to_world()  # nothing moved in between

        assert world.state.version == version_before
    finally:
        stop_multisim_if_running(multi_sim)


def test_on_state_change_logs_when_no_mujoco_joint_is_found(monkeypatch):
    """
    Regression test: both sync directions used to silently skip a connection whenever
    _resolve_qpos_adr found no matching MuJoCo joint, with no log line - unlike the
    neighboring "unsupported connection type" branch, which does warn. A spawn failure
    or a connection/joint name mismatch was therefore invisible.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        dof = _spawn_revolute_joint(world, "missing_joint_test")
        monkeypatch.setattr(
            multi_sim.synchronizer, "_resolve_qpos_adr", lambda connection: None
        )

        with _RecordingLogHandler(
            "semantic_digital_twin.adapters.multi_sim"
        ) as log_handler:
            world.state[dof.id].position = 0.3
            world.notify_state_change()

        assert log_handler.has_message_containing("no MuJoCo joint found")
    finally:
        stop_multisim_if_running(multi_sim)


def test_sim_to_world_logs_when_no_mujoco_joint_is_found(monkeypatch):
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        _spawn_revolute_joint(world, "missing_joint_test_2")
        monkeypatch.setattr(
            multi_sim.synchronizer, "_resolve_qpos_adr", lambda connection: None
        )

        multi_sim.synchronizer.sync_rate_hz = MujocoSynchronizer.UNTHROTTLED_SYNC_RATE_HZ
        with _RecordingLogHandler(
            "semantic_digital_twin.adapters.multi_sim"
        ) as log_handler:
            multi_sim.synchronizer._sim_to_world()

        assert log_handler.has_message_containing("no MuJoCo joint found")
    finally:
        stop_multisim_if_running(multi_sim)


def test_multiple_synchronizers_can_share_one_simulator():
    """
    Regression test: MujocoSynchronizer used to monkeypatch
    simulator.read_data_from_simulator as a single instance attribute - a second
    synchronizer attaching to the same simulator silently overwrote the first's hook,
    and stopping either synchronizer deleted the sole shared attribute regardless of
    which one owned it, breaking the other synchronizer.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        simulator = multi_sim.simulator
        first_synchronizer = multi_sim.synchronizer
        second_synchronizer = MujocoSynchronizer(_world=world, simulator=simulator)
        try:
            assert first_synchronizer._sim_to_world in simulator._data_read_hooks
            assert second_synchronizer._sim_to_world in simulator._data_read_hooks

            second_synchronizer.stop()

            assert first_synchronizer._sim_to_world in simulator._data_read_hooks
            assert second_synchronizer._sim_to_world not in simulator._data_read_hooks
        finally:
            if second_synchronizer._sim_to_world in simulator._data_read_hooks:
                second_synchronizer.stop()
    finally:
        stop_multisim_if_running(multi_sim)


def test_resolve_qpos_adr_is_cached_and_invalidated_on_model_change(monkeypatch):
    """
    Regression test: _resolve_qpos_adr called mujoco.mj_name2id (a string joint-name
    lookup) for every connection on every sync call, in both directions, instead of
    caching the resolved qpos address. This asserts a second lookup for the same
    connection is served from cache, and that a later model change (which can shift
    every joint's qpos address after a recompile) invalidates that cache.

    Adding a connection also triggers the framework's own automatic state resync
    (World._notify_model_change calls notify_state_change right after notifying model
    callbacks), which itself re-resolves every connection's qpos address as a side
    effect - so this checks *which joint names* mj_name2id was asked to resolve around
    the second spawn, rather than a raw call count, to isolate what this test is
    actually checking: whether joint_1's now-stale cache entry gets looked up again.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        dof_1 = _spawn_revolute_joint(world, "cache_test_joint_1")
        connection_1 = next(
            c for c in world.connections if getattr(c, "raw_dof", None) is dof_1
        )

        looked_up_names = []
        original_mj_name2id = mujoco.mj_name2id

        def recording_mj_name2id(*args, **kwargs):
            looked_up_names.append(args[2] if len(args) > 2 else kwargs.get("name"))
            return original_mj_name2id(*args, **kwargs)

        monkeypatch.setattr(mujoco, "mj_name2id", recording_mj_name2id)

        multi_sim.synchronizer._qpos_adr_cache.clear()
        looked_up_names.clear()
        multi_sim.synchronizer._resolve_qpos_adr(connection_1)
        multi_sim.synchronizer._resolve_qpos_adr(connection_1)
        assert looked_up_names == [
            "cache_test_joint_1"
        ], "second lookup for the same connection was not served from cache"

        looked_up_names.clear()
        _spawn_revolute_joint(world, "cache_test_joint_2")  # a real model change

        assert "cache_test_joint_1" in looked_up_names, (
            "cache_test_joint_1's now-stale cache entry was not invalidated by the "
            "model change"
        )
    finally:
        stop_multisim_if_running(multi_sim)


def test_connection6dof_with_non_root_parent_raises_instead_of_silently_wrong_sync():
    """
    Regression test: MujocoSynchronizer's 6DoF sync assumed a Connection6DoF's parent
    is always the world root. MuJoCo always expresses a free joint's qpos directly in
    the world frame, so converting it into the connection's own dofs needs to also
    fold in the parent's own pose whenever the parent isn't the world root - silently
    skipping that produced a wrong pose instead of failing loudly.

    In practice MuJoCo's own compiler already refuses to build a free joint that isn't
    a direct child of the top-level body ("free joint can only be used on top level"),
    so a Connection6DoF with a non-root parent can never actually reach a live
    MujocoSimulator through the normal build/spawn path - this exercises the
    synchronizer's guard directly instead, so the assumption still fails loudly if that
    ever changes (e.g. via body merging in the builder) rather than silently producing
    a wrong pose.
    """
    world = World()
    multi_sim = MujocoSim(world=world, headless=headless, step_size=STEP_SIZE)
    try:
        intermediate_body = Body(name=PrefixedName("intermediate"))
        floating_body = Body(name=PrefixedName("floating_child"))
        with world.modify_world():
            world.add_connection(
                FixedConnection(parent=world.root, child=intermediate_body)
            )
            connection = Connection6DoF.create_with_dofs(
                world=world,
                parent=intermediate_body,
                child=floating_body,
            )
            # deliberately not world.add_connection(connection): see docstring above.

        with pytest.raises(UnsupportedConnection6DoFParentError):
            multi_sim.synchronizer._read_6dof_from_qpos(connection, qpos_adr=0)
    finally:
        stop_multisim_if_running(multi_sim)

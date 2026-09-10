"""
Stage 1: physics-verified grasp trials, now over three real geometries (Cube, milk
carton, cup -- see domain_model.GraspableKind) instead of just the cube.

`approach_pose` is a real SE(3) Pose of the gripper ("hand") tip relative to the
Panda's `link0`, solved by Giskard (see giskard_reach.py) right before physics
execution -- not a hardcoded joint vector. The Giskard solve runs with
ExternalCollisionAvoidance enabled (coraplex's collision-avoidance flag, mirrored here
-- see giskard_reach.solve_reach_trajectory) for the whole reach, with the two finger
bodies exempted from the collision matrix (cram2 PR #589's
AllowCollisionForEndEffector, mirroring coraplex's allow_gripper_collision) so they --
and only they -- are free to approach and straddle the object. *Every intermediate joint
configuration* Giskard passes through is replayed into MuJoCo as a waypoint, not just
the final one -- collision avoidance only protects Giskard's own solve, so discarding
the path and jumping a separate PD controller straight to the endpoint would still be
free to clip anything in between. The object sits at its real table position for the
whole run; the earlier "teleport it into the gripper after the arm settles" hack is gone.

Because a Pose's `reference_frame` is a specific Body belonging to one World instance,
the world must be built once and shared between whoever constructs the candidate's
approach_pose and whoever executes it; `run_fixed_candidate` therefore takes an
already-built GraspWorld rather than building its own. `end_effector` is still left
None: the Panda in this fixture is loaded from raw MJCF rather than a semdt
Robot/EndEffector class, so there is no EndEffector instance to attach.
"""
import sys, os, math
sys.path.insert(0, "experiments/src")

import numpy as np

from experiments.graspability_learning.domain_model import (
    Cube, GraspableMilk, GraspableCup, GraspableKind, GraspCandidate, GraspTrialResult,
    IsGraspable,
)
from experiments.graspability_learning.giskard_reach import solve_reach_trajectory
from semantic_digital_twin.adapters.mjcf import MJCFParser
from semantic_digital_twin.adapters.multi_sim import MujocoBuilder, MujocoGeom
from semantic_digital_twin.world_description.geometry import Scale, Box, Color, Mesh
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix, Pose, Point3, Quaternion,
)
from physics_simulators.mujoco_simulator import MujocoSimulator

MJCF_PATH = "semantic_digital_twin/resources/mjcf/mjx_single_cube_no_mesh.xml"
# PID-suffixed: parallel dataset-generation workers (generate_dataset_worker.py) are
# separate OS processes that each call run_fixed_candidate repeatedly -- a single
# shared path here let two workers' write-then-read of the exported scene race each
# other, observed directly as `ValueError: ParseXML: empty file` when one process
# read the file mid-truncation by another.
SCENE_OUT = f"/tmp/scene_stage1_{os.getpid()}.xml"
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), "resources")

# Every object is placed at the same reachable table point -- only the vertical
# placement/grasp target differs per geometry (see _mesh_vertical_extent below).
OBJECT_X = 0.531
OBJECT_Y = 0.0


def _mesh_vertical_extent(filename: str) -> tuple[float, float]:
    """
    :return: (min_z, max_z) of a mesh's own local bounding box, read via semdt's own
        Mesh API (the same one robokudo's world_iai_kitchen20 descriptor uses for
        object placement) -- avoids hand-measuring/hardcoding per-mesh numbers that
        would silently go stale if a mesh asset ever changed.
    """
    mesh = Mesh(
        origin=HomogeneousTransformationMatrix(),
        filename=os.path.join(RESOURCES_DIR, filename),
    )
    bb = mesh.local_frame_bounding_box
    return bb.min_z, bb.max_z


_MILK_MIN_Z, _MILK_MAX_Z = _mesh_vertical_extent("milk.stl")
_CUP_MIN_Z, _CUP_MAX_Z = _mesh_vertical_extent("jeroen_cup.stl")

# Placement Z (world_root_T_self.z) puts each object's mesh-local bounding-box floor
# exactly on the table (world Z=0), whatever that mesh's own origin convention is.
MILK_PLACEMENT_Z = -_MILK_MIN_Z
CUP_PLACEMENT_Z = -_CUP_MIN_Z

# Grasp height (world Z of the object's own vertical center) -- always half the
# object's total vertical extent once its base sits on the table, regardless of
# where the mesh's own local origin happens to be.
MILK_GRASP_HEIGHT = (_MILK_MAX_Z - _MILK_MIN_Z) / 2
CUP_GRASP_HEIGHT = (_CUP_MAX_Z - _CUP_MIN_Z) / 2

# A safe overhead joint configuration (no collision risk) used both to settle the
# physical MuJoCo arm before the grasp approach, and as the seed configuration for
# Giskard's reactive Cartesian IK solve: seeding from zero/home sends the QP down a
# different kinematic branch that gets stuck against a joint limit (a local minimum,
# confirmed by direct inspection -- not a solver bug), while seeding near the target's
# branch converges cleanly.
HOVER_ARM = [0.0, 0.30, 0.0, -1.57079, 0.0, 2.00, -0.7853]

# Kept only to derive default_grasp_pose's target ORIENTATION (and X/Y reach point) via
# forward kinematics; the Z component is overridden per-object (see default_grasp_pose)
# since this joint vector was hand-tuned for the cube's own grasp height specifically.
GRASP_ARM = [0.0, 0.62, 0.0, -2.08, 0.0, 2.57, -0.7853]

ARM_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]

# The "hand" frame GRASP_ARM's own forward kinematics lands at (z=0.11284) is NOT the
# cube's own grasp height (z=0.02, IsGraspable.grasp_height's "object center") -- the
# Panda's fingers extend well below the "hand" frame's origin, so there is a fixed,
# gripper-geometry offset between the two, verified directly by computing GRASP_ARM's
# raw FK (0.11284) against the cube's known grasp_height (0.02). default_grasp_pose
# adds this same offset to every object's own grasp_height rather than targeting
# grasp_height directly, which would plant the "hand" frame at the object's center
# instead of the fingers.
_CUBE_RAW_FK_Z = 0.11284
_CUBE_GRASP_HEIGHT = 0.02
GRIPPER_Z_OFFSET = _CUBE_RAW_FK_Z - _CUBE_GRASP_HEIGHT

# Giskard's QPControllerConfig(target_frequency=50) ticks at 0.02s per control cycle;
# 10 MuJoCo steps of 0.002s each match that exactly, so replaying one trajectory
# waypoint per Giskard tick keeps the physical replay time-accurate to the solve.
_MUJOCO_STEPS_PER_WAYPOINT = 10


class GraspWorld:
    """
    Everything run_fixed_candidate needs beyond the candidate itself: the shared
    World, the graspable object living in it, the MuJoCo body name to track, and the
    world-Z its base rests at (used to turn the absolute lift height the cube
    experiment was tuned against into a per-object *relative* rise -- see
    run_fixed_candidate's success check).

    Deliberately a plain class, not a @dataclass: generate_orm.py's ORMatic scan
    walks every dataclass in the `experiments` package looking for things to map to
    a DB table, and this is pure runtime bookkeeping (its `world` field isn't even a
    typed, mappable field) -- a dataclass here made that scan crash with
    "Could not locate SQLAlchemy Core type ... for 'world'" the moment this file was
    imported anywhere in the package, not just when actually persisting one.
    """

    def __init__(self, world, graspable: IsGraspable, body_name: str, rest_z: float):
        self.world = world
        self.graspable = graspable
        self.body_name = body_name
        self.rest_z = rest_z


def _add_pad_fingers(world) -> None:
    for finger_name in ["left_finger", "right_finger"]:
        finger = world.get_body_by_name(finger_name)
        pad = Box(
            scale=Scale(0.02, 0.012, 0.03),
            color=Color(0.2, 0.2, 0.2, 1.0),
            origin=HomogeneousTransformationMatrix.from_xyz_rpy(reference_frame=finger),
        )
        pad.simulator_additional_properties.append(MujocoGeom(friction=[1.0, 0.005, 0.0001]))
        finger.collision = ShapeCollection([pad], reference_frame=finger)


def build_world(kind: GraspableKind = GraspableKind.CUBE) -> GraspWorld:
    """
    Build the semdt World: the fixture's Panda (no finger collision geoms in this
    variant, so we add a pad Box to each finger) + one graspable object of the
    requested kind, spawned at its real table position from the start -- the
    collision-avoiding reach (see run_fixed_candidate) is what's responsible for not
    hitting it, rather than keeping it out of the way until the arm has already
    settled.
    """
    world = MJCFParser(MJCF_PATH).parse()
    world.remove_branch_from_world(world.get_body_by_name("box"))
    _add_pad_fingers(world)

    if kind is GraspableKind.CUBE:
        body_name = "box"
        rest_z = 0.02
        graspable = Cube.create_with_new_body_in_world(
            body_name,
            world,
            # The cube mesh is 0.04 x 0.04 x 0.06 (taller than its own base) -- spawned
            # standing upright it is physically unstable and tips onto its side within
            # ~0.2s of gravity settling alone, before the arm ever moves (verified by
            # instrumentation). Spawning it already lying on its side (z=0.02, half of
            # the 0.04 dimension that is now vertical) avoids that.
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=OBJECT_X, y=OBJECT_Y, z=rest_z, pitch=-math.pi / 2, reference_frame=world.root
            ),
            scale=Scale(1.0, 1.0, 1.0),
        )
        graspable.root.inertial.mass = 0.05
        friction = [1.0, 0.03, 0.003]
    elif kind is GraspableKind.MILK:
        body_name = "milk"
        rest_z = MILK_PLACEMENT_Z
        graspable = GraspableMilk.create_with_new_body_in_world(
            body_name,
            world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=OBJECT_X, y=OBJECT_Y, z=rest_z, reference_frame=world.root
            ),
            scale=Scale(1.0, 1.0, 1.0),
        )
        # A real 1L carton is close to 1kg full; kept lighter (partially full) so the
        # panda's small parallel gripper has a realistic chance of holding it.
        graspable.root.inertial.mass = 0.3
        friction = [1.0, 0.02, 0.002]
    elif kind is GraspableKind.CUP:
        body_name = "cup"
        rest_z = CUP_PLACEMENT_Z
        graspable = GraspableCup.create_with_new_body_in_world(
            body_name,
            world,
            world_root_T_self=HomogeneousTransformationMatrix.from_xyz_rpy(
                x=OBJECT_X, y=OBJECT_Y, z=rest_z, reference_frame=world.root
            ),
            scale=Scale(1.0, 1.0, 1.0),
        )
        graspable.root.inertial.mass = 0.08
        friction = [1.0, 0.02, 0.002]
    else:
        raise ValueError(f"Unknown GraspableKind: {kind}")

    shape = list(graspable.root.collision)[0]
    shape.simulator_additional_properties.append(MujocoGeom(friction=friction))
    return GraspWorld(world=world, graspable=graspable, body_name=body_name, rest_z=rest_z)


def default_grasp_pose(world, grasp_height: float, level: bool = False) -> Pose:
    """
    :param grasp_height: World-frame Z the gripper should target -- everything else
        (X/Y reach point, orientation) comes from GRASP_ARM's forward kinematics,
        which only depended on the object sitting at OBJECT_X/OBJECT_Y in the first
        place, not on its height.
    :param level: GRASP_ARM's raw FK orientation carries a deliberate extra roll,
        needed because the cube itself is placed pitched -90 degrees (lying on its
        side, see build_world) -- the tilt is what aligns the gripper with the
        cube's *actual*, rotated faces, not an accidental artifact (confirmed
        directly: removing it for the cube regresses a working grasp, slip
        0.024->0.199). Milk/cup are placed upright with no such rotation, so they
        need the *level* (de-tilted) orientation instead -- confirmed directly the
        other way: the raw tilted orientation left milk's two fingers ~1.6cm apart
        in height (0-2 contacts, tight margin against its ~3.25cm half-width vs the
        gripper's 4cm hard limit), while levelling it dropped that gap to ~0.5cm
        (4+ contacts). Pass True for upright objects, False (default) for the cube.
    :return: The Pose a physics-verified fixed joint configuration (GRASP_ARM)
        achieves in this exact world, with its Z overridden to grasp_height and its
        orientation optionally levelled.
    """
    root = world.get_body_by_name("link0")
    tip = world.get_body_by_name("hand")
    saved = {
        name: world.state[world.get_connection_by_name(name).dof.id].position
        for name in ARM_JOINT_NAMES
    }
    for name, angle in zip(ARM_JOINT_NAMES, GRASP_ARM):
        world.state[world.get_connection_by_name(name).dof.id].position = angle
    world.notify_state_change()
    # world.compute_forward_kinematics returns a plain HomogeneousTransformationMatrix,
    # not a Pose -- they're sibling types (both SpatialType/SubclassJSONSerializer, but
    # neither subclasses the other), and GraspCandidate.approach_pose is typed as Pose.
    htm = world.compute_forward_kinematics(root, tip)
    position = htm.to_position()
    raw_orientation = htm.to_quaternion()
    if level:
        tilt_norm = math.sqrt(raw_orientation.x ** 2 + raw_orientation.y ** 2)
        orientation = Quaternion(
            w=0.0, x=raw_orientation.x / tilt_norm, y=raw_orientation.y / tilt_norm, z=0.0
        )
    else:
        orientation = raw_orientation
    target_z = grasp_height + GRIPPER_Z_OFFSET
    pose = Pose(
        position=Point3(x=position.x, y=position.y, z=target_z, reference_frame=position.reference_frame),
        orientation=orientation,
        reference_frame=htm.reference_frame,
    )
    for name, angle in saved.items():
        world.state[world.get_connection_by_name(name).dof.id].position = angle
    world.notify_state_change()
    return pose


def solve_approach_trajectory(world, approach_pose: Pose) -> list:
    """
    Solve `approach_pose` to a joint trajectory via Giskard, seeded from HOVER_ARM (see
    module docstring for why the seed matters). `approach_pose` must belong to the same
    World instance (its `reference_frame` is a specific Body of it).

    One collision-avoiding solve for the whole reach, with the two finger bodies
    exempted from the collision matrix via cram2 PR #589's AllowCollisionForEndEffector
    (see giskard_reach.solve_reach_trajectory's end_effector_bodies parameter) --
    mirroring coraplex's own allow_gripper_collision flag.

    :return: A list of 7-vectors (arm joint angles), one per Giskard control tick, from
        just after HOVER_ARM to the solved goal.
    """
    root = world.get_body_by_name("link0")
    tip = world.get_body_by_name("hand")
    for name, angle in zip(ARM_JOINT_NAMES, HOVER_ARM):
        world.state[world.get_connection_by_name(name).dof.id].position = angle
    world.notify_state_change()

    finger_bodies = [
        world.get_body_by_name("left_finger"),
        world.get_body_by_name("right_finger"),
    ]
    trajectory = solve_reach_trajectory(
        world, tip, approach_pose, root=root, timeout=2000,
        collision_avoidance=True, end_effector_bodies=finger_bodies,
    )
    return [[waypoint[name] for name in ARM_JOINT_NAMES] for waypoint in trajectory]


def run_fixed_candidate(candidate: GraspCandidate, grasp_world: GraspWorld) -> GraspTrialResult:
    """
    :param grasp_world: The GraspWorld `candidate.approach_pose` was built against
        (build_world()'s return value) -- carries the world, the tracked body's name,
        and its resting Z (see the relative success check below).
    """
    world = grasp_world.world
    trajectory = solve_approach_trajectory(world, candidate.approach_pose)
    arm_angles = trajectory[-1]

    # solve_approach_trajectory leaves world.state at the SOLVED grasp pose (Giskard's
    # own IK stepping mutates it in place as a side effect) -- MujocoBuilder bakes
    # whatever world.state currently is as the exported scene's initial qpos. Baking
    # the grasp pose itself, rather than a collision-free configuration, produces a
    # violent interpenetration contact impulse on the very first simulated step for
    # large objects: verified directly for milk (its solved grasp configuration
    # already overlaps its own collision volume), where this flung the carton several
    # centimeters away before the actuators below even had a chance to move the arm
    # back to HOVER_ARM -- identical slip/contacts across wildly different aperture/
    # closing_effort was the tell, since none of those settings could matter if the
    # object had already been launched away before Phase B closes the fingers. The
    # cube's smaller size and lower grasp height happened not to trigger this, which
    # is why it went unnoticed until a taller, wider object exposed it. Resetting to
    # HOVER_ARM before exporting bakes a safe starting pose instead; the actual
    # approach is still replayed waypoint-by-waypoint via set_ctrl below, unchanged.
    for name, angle in zip(ARM_JOINT_NAMES, HOVER_ARM):
        world.state[world.get_connection_by_name(name).dof.id].position = angle
    world.notify_state_change()

    MujocoBuilder().build_world(world=world, file_path=SCENE_OUT)

    sim = MujocoSimulator(_headless=True, _step_size=0.002, file_path=SCENE_OUT)
    sim.start(simulate_in_thread=False, render_in_thread=False)

    def set_ctrl(arm_angles, finger_target):
        for i, angle in enumerate(arm_angles):
            sim.get_actuator(f"actuator{i + 1}").result.ctrl[0] = angle
        sim.get_actuator("actuator8").result.ctrl[0] = finger_target

    open_width = candidate.aperture
    body_name = grasp_world.body_name

    # Phase A: settle at a safe hover pose, then replay Giskard's collision-avoiding
    # reach trajectory waypoint by waypoint (fingers open) -- not a single jump to the
    # final target. The object sits at its real table position the whole time; nothing
    # teleports it in after the fact.
    set_ctrl(HOVER_ARM, open_width)
    for _ in range(400):
        sim.step()
    for waypoint in trajectory:
        set_ctrl(waypoint, open_width)
        for _ in range(_MUJOCO_STEPS_PER_WAYPOINT):
            sim.step()
    # Let the arm settle at the final reach pose before closing.
    for _ in range(300):
        sim.step()

    # Phase B: close fingers. closing_effort scales the finger actuator's force limit.
    finger_actuator_id = sim.get_all_actuator_names().result.index("actuator8")
    sim._mj_model.actuator_forcerange[finger_actuator_id] = [
        -candidate.closing_effort, candidate.closing_effort,
    ]
    close_target = 0.005
    set_ctrl(arm_angles, close_target)
    for _ in range(600):
        sim.step()
    grasp_offset_ref = np.array(sim.get_body_position(body_name).result) - np.array(
        sim.get_body_position("hand").result
    )

    # Phase C: lift, holding the closed grip, and track slip.
    lift_arm = list(arm_angles)
    lift_arm[1] -= 0.35
    set_ctrl(lift_arm, close_target)
    object_positions, hand_positions = [], []
    n_hold_steps = 1500
    for _ in range(n_hold_steps):
        sim.step()
        object_positions.append(np.array(sim.get_body_position(body_name).result))
        hand_positions.append(np.array(sim.get_body_position("hand").result))

    contacts = sim._mj_data.ncon
    offsets = [
        p_obj - p_hand - grasp_offset_ref
        for p_obj, p_hand in zip(object_positions, hand_positions)
    ]
    max_translation_slip = float(max(np.linalg.norm(o) for o in offsets))
    # Relative rise, not an absolute Z threshold: the cube's own resting Z (0.02) is
    # not comparable to milk's (~0.088) or the cup's (0.0), so "did it end up
    # meaningfully higher than where it started" is the geometry-agnostic version of
    # the cube-only check this replaces (which required box_positions[-1][2] > 0.15,
    # i.e. a rise of > 0.13 over its 0.02 rest height).
    success = bool(object_positions[-1][2] - grasp_world.rest_z > 0.13)

    sim.stop()

    return GraspTrialResult(
        candidate=candidate,
        success=success,
        max_translation_slip=max_translation_slip,
        max_rotation_slip=0.0,
        contact_count=contacts,
        held_duration=n_hold_steps * sim.step_size,
    )


if __name__ == "__main__":
    grasp_world = build_world(GraspableKind.CUBE)
    approach_pose = grasp_world.graspable._default_approach_pose()
    print("APPROACH POSE:", approach_pose.to_position(), approach_pose.to_quaternion())
    fixed_candidate = GraspCandidate(
        graspable=grasp_world.graspable,
        end_effector=None,
        approach_pose=approach_pose,
        aperture=0.04,
        closing_effort=40.0,
        graspable_kind=GraspableKind.CUBE,
    )
    result = run_fixed_candidate(fixed_candidate, grasp_world)
    print("RESULT:", result)

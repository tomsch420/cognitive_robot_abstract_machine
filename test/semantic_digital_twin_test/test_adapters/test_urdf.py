import os.path
from dataclasses import dataclass

import pytest

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.robots.pr2 import PR2
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Color, Mesh


@dataclass
class URDFPaths:
    """
    Data class to hold paths to URDF files used in tests.
    """

    table: str
    kitchen: str
    apartment: str
    pr2: str


@pytest.fixture
def urdf_paths():
    """
    Fixture providing paths to various URDF files.
    """
    urdf_directory = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "..",
        "..",
        "semantic_digital_twin",
        "resources",
        "urdf",
    )
    return URDFPaths(
        table=os.path.join(urdf_directory, "table.urdf"),
        kitchen=os.path.join(urdf_directory, "kitchen-small.urdf"),
        apartment=os.path.join(urdf_directory, "apartment.urdf"),
        pr2=PR2.get_ros_file_path(),
    )


@pytest.fixture
def table_parser(urdf_paths):
    """
    Fixture providing a URDFParser for the table model.
    """
    return URDFParser.from_file(file_path=urdf_paths.table)


@pytest.fixture
def kitchen_parser(urdf_paths):
    """
    Fixture providing a URDFParser for the kitchen model.
    """
    return URDFParser.from_file(file_path=urdf_paths.kitchen)


@pytest.fixture
def apartment_parser(urdf_paths):
    """
    Fixture providing a URDFParser for the apartment model.
    """
    return URDFParser.from_file(file_path=urdf_paths.apartment)


@pytest.fixture
def pr2_parser():
    """
    Fixture providing a URDFParser for the PR2 model.
    """
    return URDFParser.from_file(file_path=PR2.get_ros_file_path())


def test_table_parsing(table_parser):
    world = table_parser.parse()
    world.validate()
    assert len(world.kinematic_structure_entities) == 6

    origin_left_front_leg_joint = world.get_connection(
        world.root, world.kinematic_structure_entities[1]
    )
    assert isinstance(origin_left_front_leg_joint, FixedConnection)


def test_kitchen_parsing(kitchen_parser):
    world = kitchen_parser.parse()
    world.validate()
    assert len(world.kinematic_structure_entities) > 0
    assert len(world.connections) > 0


def test_apartment_parsing(apartment_parser):
    world = apartment_parser.parse()
    world.validate()
    assert len(world.kinematic_structure_entities) > 0
    assert len(world.connections) > 0


def test_pr2_parsing(pr2_parser):
    world = pr2_parser.parse()
    world.validate()
    assert len(world.kinematic_structure_entities) > 0
    assert len(world.connections) > 0
    assert world.root.name.name == "base_footprint"


def test_mimic_joints(pr2_parser):
    world = pr2_parser.parse()
    joint_to_be_mimicked = world.get_connection_by_name("l_gripper_l_finger_joint")
    mimic_joint = world.get_connection_by_name("l_gripper_r_finger_joint")

    assert joint_to_be_mimicked.dofs == mimic_joint.dofs


def test_revolute_joint_with_near_unit_axis_is_not_truncated_to_zero():
    # a real-world axis is rarely an exact integer, e.g. this is l_shoulder_pitch's
    # axis from the iCub3 URDF
    urdf = """<?xml version="1.0"?>
    <robot name="axis_test">
      <link name="parent_link"/>
      <link name="child_link"/>
      <joint name="test_joint" type="revolute">
        <parent link="parent_link"/>
        <child link="child_link"/>
        <axis xyz="3.323079797554136e-07 0.9999999783534342 1.4452224303420037e-07"/>
        <limit lower="-1.0" upper="1.0" effort="1.0" velocity="1.0"/>
      </joint>
    </robot>
    """
    world = URDFParser(urdf=urdf).parse()

    connection = world.get_connection_by_name("test_joint")

    assert connection.axis.to_np()[:3] == pytest.approx([0.0, 1.0, 0.0], abs=1e-6)


def test_xacro():
    path = "package://iai_pr2_description/robots/pr2_with_ft2_cableguide.xacro"
    parser = URDFParser.from_xacro(path)
    world = parser.parse()
    world.validate()
    assert len(world.kinematic_structure_entities) > 0
    assert len(world.connections) > 0
    assert world.root.name.name == "base_footprint"


@pytest.fixture
def milk_mesh_file_path(urdf_paths):
    """
    Fixture providing the path to a small mesh file usable as a URDF visual geometry.
    """
    return os.path.join(
        os.path.dirname(os.path.abspath(urdf_paths.table)),
        "..",
        "stl",
        "milk.stl",
    )


@pytest.fixture
def globally_declared_material_urdf_path(milk_mesh_file_path, tmp_path):
    """
    Fixture providing a URDF whose mesh visual references a material declared once on
    the ``<robot>`` root by name, the way ``iai_tracy_description`` and
    ``iai_kitchen`` describe their meshes.
    """
    urdf = f"""<?xml version="1.0"?>
<robot name="colored_mesh_robot">
  <material name="milk_material">
    <color rgba="0.8 0.2 0.1 1.0"/>
  </material>
  <link name="milk_carton">
    <visual>
      <geometry>
        <mesh filename="{milk_mesh_file_path}"/>
      </geometry>
      <material name="milk_material"/>
    </visual>
  </link>
</robot>
"""
    urdf_path = tmp_path / "colored_mesh_robot.urdf"
    urdf_path.write_text(urdf)
    return str(urdf_path)


@pytest.fixture
def inline_material_urdf_path(milk_mesh_file_path, tmp_path):
    """
    Fixture providing a URDF whose mesh visual defines its material's color inline,
    without declaring it on the ``<robot>`` root, the way the Montessori board scene
    describes its meshes.
    """
    urdf = f"""<?xml version="1.0"?>
<robot name="colored_mesh_robot">
  <link name="milk_carton">
    <visual>
      <geometry>
        <mesh filename="{milk_mesh_file_path}"/>
      </geometry>
      <material name="milk_material">
        <color rgba="0.8 0.2 0.1 1.0"/>
      </material>
    </visual>
  </link>
</robot>
"""
    urdf_path = tmp_path / "colored_mesh_robot.urdf"
    urdf_path.write_text(urdf)
    return str(urdf_path)


def test_mesh_visual_color_from_globally_declared_material_is_parsed(
    globally_declared_material_urdf_path,
):
    world = URDFParser.from_file(
        file_path=globally_declared_material_urdf_path
    ).parse()
    body = world.get_body_by_name("milk_carton")

    mesh_shape = body.visual.shapes[0]
    assert isinstance(mesh_shape, Mesh)
    assert mesh_shape.color == Color(0.8, 0.2, 0.1, 1.0)


def test_mesh_visual_color_from_inline_material_is_parsed(inline_material_urdf_path):
    world = URDFParser.from_file(file_path=inline_material_urdf_path).parse()
    body = world.get_body_by_name("milk_carton")

    mesh_shape = body.visual.shapes[0]
    assert isinstance(mesh_shape, Mesh)
    assert mesh_shape.color == Color(0.8, 0.2, 0.1, 1.0)

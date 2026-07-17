import pytest

from experiments.montessori.montessori_demo import (
    ARM_ACTUATOR_POSITION_GAIN,
    BASE_ACTUATOR_POSITION_GAIN,
    BASE_JOINT_DAMPING,
    BASE_JOINT_DRY_FRICTION,
    _base_connections_without_hardware_interface,
    _base_degrees_of_freedom_without_hardware_interface,
    _hold_controlled_joints_in_mujoco,
    _make_all_shapes_movable_in_mujoco,
    _make_shape_movable_in_mujoco,
)
from experiments.montessori.semantics import MontessoriShape
from experiments.montessori.world import MontessoriWorld
from semantic_digital_twin.utils import hsrb_installed
from semantic_digital_twin.world_description.connections import Connection6DoF

EXPECTED_BASE_DOF_NAMES = {
    "base_roll_joint",
    "base_l_passive_wheel_x_frame_joint",
    "base_l_passive_wheel_y_frame_joint",
    "base_l_passive_wheel_z_joint",
    "base_r_passive_wheel_x_frame_joint",
    "base_r_passive_wheel_y_frame_joint",
    "base_r_passive_wheel_z_joint",
    "base_l_drive_wheel_joint",
    "base_r_drive_wheel_joint",
}


@pytest.fixture
def montessori_with_hsrb():
    if not hsrb_installed():
        pytest.skip("hsr_description is not installed")

    montessori = MontessoriWorld()
    montessori.spawn_hsrb()
    return montessori


def test_base_degrees_of_freedom_without_hardware_interface_finds_the_wheel_joints(
    montessori_with_hsrb,
):
    hsrb = montessori_with_hsrb.hsrb
    base_dof_names = {
        dof.name.name
        for dof in _base_degrees_of_freedom_without_hardware_interface(hsrb)
    }

    assert base_dof_names == EXPECTED_BASE_DOF_NAMES


def test_base_degrees_of_freedom_without_hardware_interface_excludes_controlled_and_gripper_dofs(
    montessori_with_hsrb,
):
    hsrb = montessori_with_hsrb.hsrb
    base_dofs = set(_base_degrees_of_freedom_without_hardware_interface(hsrb))

    assert base_dofs.isdisjoint(hsrb.degrees_of_freedom_with_hardware_interface)
    assert not any(dof.name.name.startswith("hand_") for dof in base_dofs)


def test_hold_controlled_joints_in_mujoco_adds_one_actuator_per_held_dof(
    montessori_with_hsrb,
):
    hsrb = montessori_with_hsrb.hsrb
    controlled_dofs = set(hsrb.degrees_of_freedom_with_hardware_interface)
    base_dofs = set(_base_degrees_of_freedom_without_hardware_interface(hsrb))

    _hold_controlled_joints_in_mujoco(hsrb)

    held_dofs = {dof for actuator in hsrb._world.actuators for dof in actuator.dofs}
    assert held_dofs == controlled_dofs | base_dofs


def test_hold_controlled_joints_in_mujoco_uses_a_weaker_gain_for_the_base(
    montessori_with_hsrb,
):
    hsrb = montessori_with_hsrb.hsrb
    base_dofs = set(_base_degrees_of_freedom_without_hardware_interface(hsrb))

    _hold_controlled_joints_in_mujoco(hsrb)

    for actuator in hsrb._world.actuators:
        [mujoco_actuator] = actuator.simulator_additional_properties
        expected_gain = (
            BASE_ACTUATOR_POSITION_GAIN
            if actuator.dofs[0] in base_dofs
            else ARM_ACTUATOR_POSITION_GAIN
        )
        assert mujoco_actuator.gain_parameters[0] == expected_gain


def test_hold_controlled_joints_in_mujoco_adds_joint_damping_to_the_base(
    montessori_with_hsrb,
):
    hsrb = montessori_with_hsrb.hsrb
    base_connections = _base_connections_without_hardware_interface(hsrb)

    _hold_controlled_joints_in_mujoco(hsrb)

    for connection in base_connections:
        assert connection.dynamics.damping == BASE_JOINT_DAMPING
        assert connection.dynamics.dry_friction == BASE_JOINT_DRY_FRICTION


def test_make_shape_movable_in_mujoco_replaces_the_fixed_connection():
    montessori = MontessoriWorld()
    [shape] = [
        shape
        for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape)
        if shape.shape_category == "sphere"
    ]

    _make_shape_movable_in_mujoco(shape, montessori.world)

    assert isinstance(shape.root.parent_connection, Connection6DoF)


def test_make_shape_movable_in_mujoco_preserves_the_shapes_pose():
    montessori = MontessoriWorld()
    [shape] = [
        shape
        for shape in montessori.world.get_semantic_annotations_by_type(MontessoriShape)
        if shape.shape_category == "sphere"
    ]
    montessori.world.update_forward_kinematics()
    position_before = shape.global_transform.to_position()

    _make_shape_movable_in_mujoco(shape, montessori.world)
    montessori.world.update_forward_kinematics()
    position_after = shape.global_transform.to_position()

    assert float(position_after.x) == pytest.approx(float(position_before.x))
    assert float(position_after.y) == pytest.approx(float(position_before.y))
    assert float(position_after.z) == pytest.approx(float(position_before.z))


def test_make_all_shapes_movable_in_mujoco_replaces_every_shapes_connection():
    montessori = MontessoriWorld()
    shapes = list(montessori.world.get_semantic_annotations_by_type(MontessoriShape))
    assert shapes

    _make_all_shapes_movable_in_mujoco(montessori)

    assert all(
        isinstance(shape.root.parent_connection, Connection6DoF) for shape in shapes
    )

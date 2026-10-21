"""
Tests for simulating the HSR physically in MuJoCo: its joints are driven by servos and
its driven parts carry their own weight.
"""

from __future__ import annotations

import math
from datetime import timedelta

import mujoco
import pytest

from ...pytest_environment import runs_in_continuous_integration

from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.datastructures.definitions import (
    GripperState,
    StaticJointState,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.robots.hsrb import HSRB, HSRBJoint
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import (
    Connection6DoF,
    FixedConnection,
)
from semantic_digital_twin.world_description.contact import ContactParameters
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import (
    Body,
    ContactExclusion,
    GravityCompensation,
    PositionServo,
    VelocityServo,
)

SETTLING_DURATION = timedelta(seconds=1)
"""
How long the HSR is left to move before its pose is read.
"""

POSITION_TOLERANCE = 0.01
"""
How far, in metres, a body may end up from where the world holds it.
"""

JOINT_TOLERANCE = 0.02
"""
How far, in radians, a driven joint may end up from its commanded position.
"""

CARTON_WIDTH = 0.06
"""
The width of a milk carton, in metres: an everyday object the hand has to close around.
"""

CARTON_HEIGHT = 0.2
"""
The height of that carton, in metres.
"""

GRASP_LIFT_HEIGHT = 0.2
"""
Where the arm lift stands while the hand closes on the carton, in metres.
"""

LIFTED_BY = 0.15
"""
How far the arm lift raises the grasped carton, in metres.
"""

SUPPORT_SIZE = 0.2
"""
The edge of the cube the carton stands on, in metres.
"""

LIFT_STEPS = 15
"""
How many steps the lift is raised in, as a controller raises it rather than in one jump
that would shake the carton loose.
"""

LIFT_STEP_DURATION = timedelta(milliseconds=50)
"""
How long each step of the lift takes.
"""

COMMANDED_ARM_FLEX = -0.5
"""
The arm flex angle a test commands, in radians: well inside the joint's range.
"""

DRIVEN_DISTANCE = 0.5
"""
How far a test drives the base, in metres.
"""

DRIVE_STEPS = 20
"""
How many control ticks the base is commanded for, as a controller commands it every
tick.
"""

DRIVE_STEP_DURATION = timedelta(milliseconds=100)
"""
How long each tick of the drive takes.
"""


@pytest.fixture
def spawned_hsr() -> HSRB:
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(Body(name=PrefixedName("floor")))
    return RobotSpecification(HSRB).spawn(world)


def driven_parts(robot: HSRB) -> list:
    """
    :return: The parts of the HSR whose joints are driven in a physical simulation.
    """
    arm = robot.all_arms[0]
    torso = robot.get_torso()
    return [arm, arm.end_effector, torso.neck, torso]


def joint_position(simulation: MujocoSim, joint_name: str) -> float:
    """
    :return: Where MuJoCo holds the joint named ``joint_name``.
    """
    return float(simulation.simulator.get_joint_value(joint_name=joint_name).result)


# %% what drives the HSR


def test_every_driven_joint_has_one_position_servo(spawned_hsr):
    world = spawned_hsr._world
    driven = {
        connection.raw_dof.id
        for part in driven_parts(spawned_hsr)
        for connection in part.servoed_connections
    }
    servoed = [
        dof.id
        for actuator in world.actuators
        if isinstance(actuator, PositionServo)
        for dof in actuator.dofs
    ]

    assert sorted(servoed, key=str) == sorted(driven, key=str)


def test_the_base_drive_has_a_velocity_servo_for_each_commanded_degree_of_freedom(
    spawned_hsr,
):
    drive = spawned_hsr.drive
    servoed = {
        dof.id
        for actuator in spawned_hsr._world.actuators
        if isinstance(actuator, VelocityServo)
        for dof in actuator.dofs
    }

    assert {dof.id for dof in drive.active_dofs} == servoed


def test_the_wheels_take_part_in_no_contact(spawned_hsr):
    """
    The base slides on its drive rather than rolling, so a wheel touching the ground
    would only drag it.
    """
    exclusions = [
        body.get_simulator_property_of_type(ContactExclusion)
        for body in spawned_hsr.mobile_base.rolling_bodies
    ]

    assert exclusions and None not in exclusions


def test_the_driven_parts_carry_their_own_weight(spawned_hsr):
    fractions = {
        body.get_simulator_property_of_type(GravityCompensation).fraction
        for part in driven_parts(spawned_hsr)
        for body in part.bodies
    }

    assert fractions == {1.0}


def test_the_open_hand_leaves_room_for_a_carton(spawned_hsr):
    world = spawned_hsr._world
    spawned_hsr.all_arms[0].end_effector.get_joint_state_by_type(
        GripperState.OPEN
    ).apply_to(world)
    world.notify_state_change()
    gap = world.collision_manager.collision_detector.check_collision_between_bodies(
        world.get_body_by_name("hand_l_distal_link"),
        world.get_body_by_name("hand_r_distal_link"),
        distance=1.0,
    ).distance

    assert gap > CARTON_WIDTH


# %% in MuJoCo


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_servos_hold_the_parked_arm_up(spawned_hsr):
    world = spawned_hsr._world
    spawned_hsr.all_arms[0].get_joint_state_by_type(StaticJointState.PARK).apply_to(
        world
    )
    world.notify_state_change()
    tool_frame = spawned_hsr.all_arms[0].end_effector.tool_frame
    parked = world.compute_forward_kinematics_np(world.root, tool_frame)[:3, 3]

    simulation = MujocoSim(world=world, headless=True)
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(SETTLING_DURATION)
        simulated = simulation.simulator.get_body_position(
            body_name=tool_frame.name.name
        ).result
    finally:
        simulation.stop_simulation()

    assert list(simulated) == pytest.approx(list(parked), abs=POSITION_TOLERANCE)


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_a_commanded_arm_joint_is_driven_there(spawned_hsr):
    world = spawned_hsr._world
    arm_flex = world.get_connection_by_name(HSRBJoint.ARM_FLEX)

    simulation = MujocoSim(world=world, headless=True)
    simulation.start_stepped_simulation()
    try:
        arm_flex.position = COMMANDED_ARM_FLEX
        simulation.step_simulation(SETTLING_DURATION)
        reached = joint_position(simulation, arm_flex.name.name)
    finally:
        simulation.stop_simulation()

    assert reached == pytest.approx(COMMANDED_ARM_FLEX, abs=JOINT_TOLERANCE)


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_gripper_closes_when_told_to(spawned_hsr):
    world = spawned_hsr._world
    gripper = spawned_hsr.all_arms[0].end_effector
    gripper.get_joint_state_by_type(GripperState.OPEN).apply_to(world)
    world.notify_state_change()
    hand_motor = world.get_connection_by_name(HSRBJoint.HAND_MOTOR)
    closed = gripper.get_joint_state_by_type(GripperState.CLOSE)

    simulation = MujocoSim(world=world, headless=True)
    simulation.start_stepped_simulation()
    try:
        closed.apply_to(world)
        world.notify_state_change()
        simulation.step_simulation(SETTLING_DURATION)
        reached = joint_position(simulation, hand_motor.name.name)
    finally:
        simulation.stop_simulation()

    assert reached == pytest.approx(hand_motor.position, abs=JOINT_TOLERANCE)


def stand_carton_between_the_open_fingers(robot: HSRB) -> Body:
    """
    Hold the hand out level in front of the HSR, open, and stand a carton on a support
    between its fingers, as a front grasp finds an object on a table.

    :return: The carton.
    """
    world = robot._world
    gripper = robot.all_arms[0].end_effector
    gripper.get_joint_state_by_type(GripperState.OPEN).apply_to(world)
    world.get_connection_by_name(HSRBJoint.ARM_FLEX).position = 0.0
    world.get_connection_by_name(HSRBJoint.WRIST_FLEX).position = -math.pi / 2
    world.get_connection_by_name(HSRBJoint.ARM_LIFT).position = GRASP_LIFT_HEIGHT
    world.notify_state_change()
    tool_position = world.compute_forward_kinematics_np(world.root, gripper.tool_frame)[
        :3, 3
    ]
    support_top = tool_position[2] - CARTON_HEIGHT / 2
    with world.modify_world():
        support = Body(name=PrefixedName("support"))
        support.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix(reference_frame=support),
                    scale=Scale(SUPPORT_SIZE, SUPPORT_SIZE, SUPPORT_SIZE),
                )
            ],
            reference_frame=support,
        )
        world.add_connection(
            FixedConnection(
                parent=world.root,
                child=support,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    *tool_position[:2],
                    support_top - SUPPORT_SIZE / 2,
                    reference_frame=world.root,
                ),
            )
        )
        carton = Body(name=PrefixedName("carton"))
        carton.collision = ShapeCollection(
            [
                Box(
                    origin=HomogeneousTransformationMatrix(reference_frame=carton),
                    scale=Scale(CARTON_WIDTH, CARTON_WIDTH, CARTON_HEIGHT),
                )
            ],
            reference_frame=carton,
        )
        world.add_connection(
            Connection6DoF.create_with_dofs(
                world=world,
                parent=world.root,
                child=carton,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    *tool_position, reference_frame=world.root
                ),
            )
        )
        ContactParameters.create_for_grasped_object().apply_to([carton])
    return carton


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_hand_holds_a_carton_it_closed_on_while_lifting_it(spawned_hsr):
    """
    MuJoCo's default friction lets a held object slide out under its own weight, so
    the simulation resolves friction exactly (elliptic cone, raised impedance ratio,
    slip removed), as a demo carrying objects does.
    """
    world = spawned_hsr._world
    carton = stand_carton_between_the_open_fingers(spawned_hsr)
    gripper = spawned_hsr.all_arms[0].end_effector
    arm_lift = world.get_connection_by_name(HSRBJoint.ARM_LIFT)

    simulation = MujocoSim(
        world=world,
        headless=True,
        cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
        impratio=10.0,
        noslip_iterations=10,
    )
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(SETTLING_DURATION)
        gripper.get_joint_state_by_type(GripperState.CLOSE).apply_to(world)
        world.notify_state_change()
        simulation.step_simulation(SETTLING_DURATION)
        grasped_at = simulation.simulator.get_body_position(
            body_name=carton.name.name
        ).result[2]
        for step in range(1, LIFT_STEPS + 1):
            arm_lift.position = GRASP_LIFT_HEIGHT + LIFTED_BY * step / LIFT_STEPS
            simulation.step_simulation(LIFT_STEP_DURATION)
        simulation.step_simulation(SETTLING_DURATION)
        lifted_to = simulation.simulator.get_body_position(
            body_name=carton.name.name
        ).result[2]
    finally:
        simulation.stop_simulation()

    assert lifted_to - grasped_at == pytest.approx(LIFTED_BY, abs=POSITION_TOLERANCE)


def drive_by(
    robot: HSRB, simulation: MujocoSim, x_offset: float, y_offset: float
) -> None:
    """
    Drive the base by the given offset from where it stands, without turning it, at the
    constant velocity that covers it in :data:`DRIVE_STEPS` ticks, commanded every tick
    as a controller does, then stop it and let it come to rest.

    :param x_offset: How far to drive along the base's facing direction, in metres.
    :param y_offset: How far to drive to its left, in metres.
    """
    world = robot._world
    drive = robot.drive
    driving_time = (DRIVE_STEPS * DRIVE_STEP_DURATION).total_seconds()
    for _ in range(DRIVE_STEPS):
        world.state[drive.x_velocity.id].velocity = x_offset / driving_time
        world.state[drive.y_velocity.id].velocity = y_offset / driving_time
        world.notify_state_change()
        simulation.step_simulation(DRIVE_STEP_DURATION)
    world.state[drive.x_velocity.id].velocity = 0.0
    world.state[drive.y_velocity.id].velocity = 0.0
    world.notify_state_change()
    simulation.step_simulation(SETTLING_DURATION)


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_base_drives_as_far_as_its_drive_is_commanded(spawned_hsr):
    world = spawned_hsr._world
    base = spawned_hsr.drive.child
    start = world.compute_forward_kinematics_np(world.root, base)[:3, 3]

    simulation = MujocoSim(world=world, headless=True)
    simulation.start_stepped_simulation()
    try:
        drive_by(spawned_hsr, simulation, x_offset=-DRIVEN_DISTANCE, y_offset=0.0)
        reached = simulation.simulator.get_body_position(
            body_name=base.name.name
        ).result
    finally:
        simulation.stop_simulation()

    expected = [start[0] - DRIVEN_DISTANCE, start[1], start[2]]
    assert list(reached) == pytest.approx(expected, abs=POSITION_TOLERANCE)


@pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)
def test_the_hand_holds_a_carton_while_the_base_drives(spawned_hsr):
    """
    The carton is held by friction alone, so it travels with the hand only if the
    grip withstands the base speeding up and slowing down.
    """
    world = spawned_hsr._world
    carton = stand_carton_between_the_open_fingers(spawned_hsr)
    gripper = spawned_hsr.all_arms[0].end_effector
    arm_lift = world.get_connection_by_name(HSRBJoint.ARM_LIFT)

    simulation = MujocoSim(
        world=world,
        headless=True,
        cone=mujoco.mjtCone.mjCONE_ELLIPTIC,
        impratio=10.0,
        noslip_iterations=10,
    )
    simulation.start_stepped_simulation()
    try:
        simulation.step_simulation(SETTLING_DURATION)
        gripper.get_joint_state_by_type(GripperState.CLOSE).apply_to(world)
        world.notify_state_change()
        simulation.step_simulation(SETTLING_DURATION)
        for step in range(1, LIFT_STEPS + 1):
            arm_lift.position = GRASP_LIFT_HEIGHT + LIFTED_BY * step / LIFT_STEPS
            simulation.step_simulation(LIFT_STEP_DURATION)
        simulation.step_simulation(SETTLING_DURATION)
        lifted = simulation.simulator.get_body_position(body_name=carton.name.name)
        lifted = list(lifted.result)
        drive_by(spawned_hsr, simulation, x_offset=-DRIVEN_DISTANCE, y_offset=0.0)
        carried = list(
            simulation.simulator.get_body_position(body_name=carton.name.name).result
        )
    finally:
        simulation.stop_simulation()

    expected = [lifted[0] - DRIVEN_DISTANCE, lifted[1], lifted[2]]
    assert carried == pytest.approx(expected, abs=POSITION_TOLERANCE)

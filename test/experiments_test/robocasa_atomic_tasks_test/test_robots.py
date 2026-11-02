"""
The robots that perform RoboCasa's atomic tasks: RoboCasa's own PandaOmron as robosuite
describes it, and choosing between it and the PR2.
"""

from __future__ import annotations

from xml.etree import ElementTree

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.panda_omron import (
    PandaOmron,
    PandaOmronDescription,
    PandaOmronJoint,
)
from experiments.robocasa_atomic_tasks.robots import (
    PandaOmronRobot,
    PR2Robot,
    RobotModel,
)
from semantic_digital_twin.datastructures.definitions import GripperState
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive

# %% fixtures


@pytest.fixture(scope="module")
def panda_omron() -> PandaOmron:
    """
    The PandaOmron, in a world of its own.
    """
    return PandaOmron.from_description()


@pytest.fixture(scope="module")
def panda_omron_document() -> ElementTree.Element:
    """
    The MJCF the PandaOmron is read from.
    """
    return ElementTree.fromstring(PandaOmronDescription().document())


# %% the PandaOmron's description


def test_the_arm_moves_on_the_seven_panda_joints(panda_omron):
    [arm] = panda_omron.all_arms

    assert [connection.name.name for connection in arm.active_connections] == list(
        PandaOmronJoint.arm()
    )


def test_the_lift_travels_as_far_as_robosuite_lets_it(
    panda_omron, panda_omron_document
):
    [lift_joint] = [
        joint
        for joint in panda_omron_document.iter("joint")
        if joint.get("name") == PandaOmronJoint.LIFT
    ]
    lower, upper = map(float, lift_joint.get("range").split())

    lift = panda_omron._world.get_connection_by_name(PandaOmronJoint.LIFT)

    assert (lift.dof.limits.lower.position, lift.dof.limits.upper.position) == (
        lower,
        upper,
    )


def test_the_base_has_no_joints_of_its_own(panda_omron, panda_omron_document):
    joint_names = {joint.get("name") for joint in panda_omron_document.iter("joint")}

    assert joint_names.isdisjoint(PandaOmronJoint.base_motion())


def test_the_fingers_part_along_the_closing_axis(panda_omron):
    world = panda_omron._world
    [arm] = panda_omron.all_arms
    gripper = arm.end_effector
    closed = gripper.get_joint_state_by_type(GripperState.CLOSE)
    opened = gripper.get_joint_state_by_type(GripperState.OPEN)
    left_tip, right_tip = (finger.tip for finger in gripper.fingers)

    def separation() -> np.ndarray:
        return (
            world.compute_forward_kinematics_np(gripper.tool_frame, left_tip)[:3, 3]
            - world.compute_forward_kinematics_np(gripper.tool_frame, right_tip)[:3, 3]
        )

    with world.reset_state_context():
        closed.apply_to(world)
        closed_separation = separation()
        opened.apply_to(world)
        opening = separation() - closed_separation

    closing_axis = gripper.closing_axis.to_np()[:3]
    np.testing.assert_allclose(np.cross(opening, closing_axis), np.zeros(3), atol=1e-9)
    assert np.linalg.norm(opening) == pytest.approx(2 * gripper.open_finger_travel)


# %% placing the robots in a kitchen


def test_the_panda_omron_stands_on_a_drive_at_the_start_pose():
    world = World.create_with_root_body()
    start_pose = HomogeneousTransformationMatrix.from_xyz_rpy(
        1.0, -0.5, 0.0, yaw=0.3, reference_frame=world.root
    )

    robot = PandaOmronRobot().place_in(world, start_pose)

    assert isinstance(robot.root.parent_connection, OmniDrive)
    np.testing.assert_allclose(
        robot.root.global_pose.to_np(), start_pose.to_np(), atol=1e-9
    )


def test_the_panda_omron_acts_with_its_only_arm(panda_omron):
    [arm] = panda_omron.all_arms

    assert PandaOmronRobot().acting_arm(panda_omron) is arm


def test_the_video_eye_is_hidden_from_the_panda_omron_video(panda_omron):
    task_robot = PandaOmronRobot()

    assert task_robot.video_eye(panda_omron) in task_robot.bodies_hidden_from_the_video(
        panda_omron
    )


# %% choosing a robot


@pytest.mark.parametrize(
    "model, task_robot_type",
    [(RobotModel.PR2, PR2Robot), (RobotModel.PANDA_OMRON, PandaOmronRobot)],
)
def test_a_robot_model_picks_its_robot(model, task_robot_type):
    assert isinstance(model.task_robot(), task_robot_type)

"""
The actions the robot performs RoboCasa's atomic tasks with.
"""

from __future__ import annotations

import inspect
from abc import ABC

import numpy as np
import pytest

from coraplex.plans.context_extensions import RobotAccess
from coraplex.robot_plans.actions.base import Action
from cramph.composites import Parallel
from cramph.context import StatechartContext
from cramph.node import StatechartNode
from cramph.statechart import Statechart
from experiments.robocasa_atomic_tasks import skills
from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron
from .conftest import KitchenOfBoxes
from giskardpy.motion_statechart.tasks.cartesian_tasks import (
    CartesianPose,
    CartesianPoseStraight,
)
from giskardpy.motion_statechart.tasks.cartesian_tasks import HoldPose
from experiments.robocasa_atomic_tasks.skills import (
    FingerApproach,
    FingerPressAction,
    FingerWithdrawAction,
    LetGoOfTheHandleAction,
    PressAction,
)
from giskardpy.motion_statechart.tasks.gripper import MoveGripper
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionForEndEffector,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.datastructures.definitions import GripperState

# %% every action


@pytest.mark.parametrize(
    "action",
    [
        candidate
        for candidate in vars(skills).values()
        if inspect.isclass(candidate)
        and issubclass(candidate, Action)
        and candidate.__module__ == skills.__name__
        and ABC not in candidate.__bases__
    ],
    ids=lambda action: action.__name__,
)
def test_every_action_can_be_made(action: type[Action]):
    assert not inspect.isabstract(action)


# %% pushing a part with one finger of the open gripper


@pytest.fixture
def lever_push() -> np.ndarray:
    """
    The way a lever beside a faucet swings at its middle: towards the robot and a little
    down.
    """
    push = np.array([0.0, -1.0, -0.07])
    return push / np.linalg.norm(push)


def tool_frame_goal_in_the_root(press: PressAction) -> np.ndarray:
    """
    :param press: A press.
    :return: The tool frame goal that puts its fingertips 10 cm before its point, in
        the world's root frame.
    """
    world = press.arm._world
    return world.transform(
        press.fingertips_goal(0.1), world.root
    ).homogeneous_matrix.to_np()


def test_every_finger_approach_spreads_the_fingers_along_the_push(lever_push):
    for finger_approach in FingerApproach.across(lever_push):
        np.testing.assert_allclose(finger_approach.closing, lever_push, atol=1e-12)
        assert finger_approach.approach @ lever_push == pytest.approx(0.0, abs=1e-12)
        assert np.linalg.norm(finger_approach.approach) == pytest.approx(1.0)


def test_finger_approaches_come_from_either_side_and_once_from_above(lever_push):
    level, other_side, from_above = FingerApproach.across(lever_push, (0.0, 90.0))

    np.testing.assert_allclose(level.approach, -other_side.approach, atol=1e-12)
    assert from_above.approach[2] < -0.99


def test_a_finger_press_opens_the_gripper():
    assert FingerPressAction.gripper_state == GripperState.OPEN
    assert PressAction.gripper_state == GripperState.CLOSE


def test_a_finger_press_puts_the_trailing_fingertip_on_the_point(panda_omron):
    [arm] = panda_omron.all_arms
    point = np.array([1.0, 0.0, 1.0])
    push = np.array([0.0, -1.0, 0.0])
    press = FingerPressAction(
        point=point, approach=np.array([-1.0, 0.0, 0.0]), closing=push, arm=arm
    )

    world_T_tool = tool_frame_goal_in_the_root(press)
    tool_P_face = press.trailing_fingertip_face().to_np()

    np.testing.assert_allclose(
        (world_T_tool @ tool_P_face)[:3], point - press.approach * 0.1, atol=1e-9
    )
    world_closing = world_T_tool[:3, :3] @ arm.end_effector.closing_axis.to_np()[:3]
    assert world_closing @ push == pytest.approx(1.0)
    assert (
        press.trailing_fingertip_face().to_np()[:3]
        @ arm.end_effector.closing_axis.to_np()[:3]
        < 0.0
    )


# %% withdrawing the finger


def straight_tool_goal_in_the_root(step: StatechartNode) -> np.ndarray:
    """
    :param step: A step moving the tool frame in a straight line, holding the tasks
        beside it or held in a parallel with them.
    :return: The pose it moves the tool frame to, in the world's root frame.
    """
    [goal] = (
        [step]
        if isinstance(step, CartesianPoseStraight)
        else [node for node in step.nodes if isinstance(node, CartesianPoseStraight)]
    )
    world = goal.kinematic_chain.tip._world
    return world.transform(goal.goal_pose, world.root).homogeneous_matrix.to_np()


def finger_withdrawal(panda_omron: PandaOmron, hold_base: bool) -> FingerWithdrawAction:
    """
    :return: A withdrawal of the open gripper from a point in front of the robot, in a
        statechart acting with the robot.
    """
    [arm] = panda_omron.all_arms
    withdrawal = FingerWithdrawAction(
        point=np.array([1.0, 0.0, 1.0]),
        approach=np.array([-1.0, 0.0, 0.0]),
        closing=np.array([0.0, -1.0, 0.0]),
        arm=arm,
        hold_base=hold_base,
    )
    Statechart(
        context=StatechartContext.with_extensions(
            world=arm._world, extensions=[RobotAccess(robot=panda_omron)]
        )
    ).add_node(node=withdrawal)
    return withdrawal


def base_holds_of(step: StatechartNode, panda_omron: PandaOmron) -> list[HoldPose]:
    """
    :return: The tasks of a step keeping the robot's base where it stands.
    """
    nodes = step.nodes if isinstance(step, Parallel) else [step]
    return [
        node
        for node in nodes
        if isinstance(node, HoldPose) and node.kinematic_chain.tip is panda_omron.root
    ]


def test_a_finger_withdrawal_rises_straight_up_once_drawn_back(
    panda_omron: PandaOmron,
):
    withdrawal = finger_withdrawal(panda_omron, hold_base=False)

    drawn_back, risen = withdrawal.create_steps()

    expected = straight_tool_goal_in_the_root(drawn_back)
    expected[2, 3] += withdrawal.rise
    np.testing.assert_allclose(
        straight_tool_goal_in_the_root(risen), expected, atol=1e-9
    )


def test_a_withdrawal_holding_the_base_holds_it_while_drawing_back_and_rising(
    panda_omron: PandaOmron,
):
    withdrawal = finger_withdrawal(panda_omron, hold_base=True)

    steps = withdrawal.create_steps()

    assert [len(base_holds_of(step, panda_omron)) for step in steps] == [1, 1]


def test_a_withdrawal_of_a_robot_moving_its_whole_body_leaves_the_base_free(
    panda_omron: PandaOmron,
):
    withdrawal = finger_withdrawal(panda_omron, hold_base=False)

    steps = withdrawal.create_steps()

    assert [len(base_holds_of(step, panda_omron)) for step in steps] == [0, 0]


# %% letting go of a handle


def tool_goal_in_the_handle_frame(
    step: StatechartNode, kitchen_of_boxes: KitchenOfBoxes
) -> np.ndarray:
    """
    :param step: A step moving the tool frame to a pose relative to the handle.
    :return: That pose, in the handle's frame.
    """
    assert isinstance(step, CartesianPose)
    assert step.goal_pose.reference_frame is kitchen_of_boxes.handle.root
    return step.goal_pose.to_np()


@pytest.fixture
def letting_go(
    panda_omron: PandaOmron, kitchen_of_boxes: KitchenOfBoxes
) -> LetGoOfTheHandleAction:
    """
    Letting go of the handle of the kitchen of boxes, held by a grasp at its middle that
    approaches it along its depth, in a statechart acting with the robot.
    """
    [arm] = panda_omron.all_arms
    handle = kitchen_of_boxes.handle
    letting_go = LetGoOfTheHandleAction(
        grasp_candidate=GraspCandidate(
            graspable=handle,
            grasp_pose=HomogeneousTransformationMatrix.from_xyz_rpy(
                reference_frame=handle.root
            ).pose,
        ),
        arm=arm,
    )
    Statechart(
        context=StatechartContext.with_extensions(
            world=arm._world, extensions=[RobotAccess(robot=panda_omron)]
        )
    ).add_node(node=letting_go)
    return letting_go


def test_letting_go_opens_the_gripper_first(letting_go: LetGoOfTheHandleAction):
    opening, _, _ = letting_go.create_steps()

    end_effector = letting_go.arm.end_effector
    assert isinstance(opening, MoveGripper)
    assert opening.width == end_effector.separation_at(
        end_effector.default_configuration(GripperState.OPEN).joint_state
    )


def test_letting_go_centres_the_tool_frame_on_the_grasp(
    letting_go: LetGoOfTheHandleAction, kitchen_of_boxes: KitchenOfBoxes
):
    _, centring, _ = letting_go.create_steps()

    end_effector = letting_go.arm.end_effector
    np.testing.assert_allclose(
        tool_goal_in_the_handle_frame(centring, kitchen_of_boxes),
        end_effector.tool_frame_goal_for_grasp(
            letting_go.grasp_candidate.grasp_pose
        ).to_np(),
        atol=1e-12,
    )


def test_letting_go_backs_straight_off_out_of_the_handle_by_the_clearance(
    letting_go: LetGoOfTheHandleAction, kitchen_of_boxes: KitchenOfBoxes
):
    _, _, backing_off = letting_go.create_steps()

    assert isinstance(backing_off, CartesianPoseStraight)
    handle_T_tool = tool_goal_in_the_handle_frame(backing_off, kitchen_of_boxes)
    tool_P_grasp = letting_go.arm.end_effector.tool_P_grasp_when_closed.to_np()
    half_depth = kitchen_of_boxes.handle_scale.x / 2
    np.testing.assert_allclose(
        (handle_T_tool @ tool_P_grasp)[:3],
        [-(half_depth + letting_go.release_clearance), 0.0, 0.0],
        atol=1e-9,
    )


def test_letting_go_allows_the_gripper_to_touch_the_handle(
    letting_go: LetGoOfTheHandleAction,
):
    [rule] = letting_go.create_temporary_collision_rules()

    assert isinstance(rule, AllowCollisionForEndEffector)
    assert rule.end_effector is letting_go.arm.end_effector

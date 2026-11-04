"""
The actions the robot performs RoboCasa's atomic tasks with.
"""

from __future__ import annotations

import inspect
from abc import ABC

import numpy as np
import pytest

from coraplex.robot_plans.actions.base import Action
from experiments.robocasa_atomic_tasks import skills
from experiments.robocasa_atomic_tasks.skills import (
    FingerApproach,
    FingerPressAction,
    PressAction,
)
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

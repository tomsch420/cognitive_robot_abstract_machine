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
    PressAction,
    SidewaysApproach,
    SidewaysPressAction,
)

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


# %% pushing a part with the sides of the fingers


@pytest.fixture
def lever_push() -> np.ndarray:
    """
    The way a lever beside a faucet swings at its middle: towards the robot and a little
    down.
    """
    push = np.array([0.0, -1.0, -0.07])
    return push / np.linalg.norm(push)


def test_every_sideways_approach_comes_in_across_the_push(lever_push):
    for sideways in SidewaysApproach.beside(lever_push):
        assert sideways.approach[:2] @ lever_push[:2] == pytest.approx(0.0, abs=1e-12)
        assert sideways.closing @ sideways.approach == pytest.approx(0.0, abs=1e-12)


def test_sideways_approaches_come_from_either_side_tilted_down_by_each_tilt(
    lever_push,
):
    tilts = (10.0, 30.0)

    approaches = [
        sideways.approach for sideways in SidewaysApproach.beside(lever_push, tilts)
    ]

    downward = [-approach[2] for approach in approaches]
    np.testing.assert_allclose(downward, np.repeat(np.sin(np.radians(tilts)), 4))
    np.testing.assert_allclose(approaches[0][:2], -approaches[2][:2])


def test_the_fingers_close_upright_before_they_close_along_the_push(lever_push):
    upright, along_the_push = SidewaysApproach.beside(lever_push, (0.0,))[:2]

    np.testing.assert_allclose(upright.closing, [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(along_the_push.closing, lever_push, atol=1e-12)


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


def test_a_sideways_press_closes_the_fingers_along_its_closing(panda_omron):
    [arm] = panda_omron.all_arms
    gripper = arm.end_effector
    approach = np.array([-1.0, 0.0, 0.0])
    closing = np.array([0.0, 0.0, 1.0])
    press = SidewaysPressAction(
        point=np.array([1.0, 0.0, 1.0]), approach=approach, closing=closing, arm=arm
    )

    rotation = tool_frame_goal_in_the_root(press)[:3, :3]

    np.testing.assert_allclose(
        rotation @ gripper.approach_axis.to_np()[:3], approach, atol=1e-9
    )
    assert abs(rotation @ gripper.closing_axis.to_np()[:3] @ closing) == pytest.approx(
        1.0
    )


def test_a_sideways_press_closing_level_rests_as_a_press_does(panda_omron):
    [arm] = panda_omron.all_arms
    point = np.array([1.0, 0.0, 1.0])
    approach = np.array([-1.0, 0.0, 0.0])
    level = np.cross([0.0, 0.0, 1.0], approach)

    sideways = SidewaysPressAction(
        point=point, approach=approach, closing=level, arm=arm
    )
    press = PressAction(point=point, approach=approach, arm=arm)

    np.testing.assert_allclose(
        tool_frame_goal_in_the_root(sideways),
        tool_frame_goal_in_the_root(press),
        atol=1e-9,
    )

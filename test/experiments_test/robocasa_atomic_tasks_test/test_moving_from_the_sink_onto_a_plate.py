"""
Tests of moving the broccoli of the sink to counter pick and place from the sink onto
the plate with the PandaOmron, from where RoboCasa spawns everything.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.exceptions import PostureDoesNotFitTheArm
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceSinkToCounter
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink_actions import TakeOutOfTheSinkParameters

# %% solving the task


def test_moving_the_broccoli_onto_the_plate_accomplishes_the_task():
    task = PickPlaceSinkToCounter(robot=RobotModel.PANDA_OMRON, physics=True)
    episode = task.compose()
    with episode.executed(physics=True):
        task.solve(episode)
        episode.judge.mirror.synchronize()

        assert episode.judge.pass_judgement().accomplished


# %% the arm's posture


def test_a_posture_with_too_few_joint_positions_is_refused():
    episode = PickPlaceSinkToCounter(robot=RobotModel.PANDA_OMRON).compose()
    parameters = TakeOutOfTheSinkParameters(arm_posture=(0.0, 0.0, 0.0))

    with pytest.raises(PostureDoesNotFitTheArm):
        parameters.posture_step(arm=episode.arm, robot=episode.robot)

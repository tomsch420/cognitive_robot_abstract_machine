"""
RoboCasa's tasks of turning a knob by the bar on its face, performed by the PandaOmron
in MuJoCo.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.oven import PreheatOven
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.stove import TurnOnStove
from experiments.robocasa_atomic_tasks.tasks import JointGoalTask

# %% turning a knob by its bar


@pytest.mark.parametrize("task_type", [PreheatOven, TurnOnStove])
def test_the_panda_omron_turns_a_knob_by_its_bar_in_physics(
    task_type: type[JointGoalTask],
):
    task = task_type(robot=RobotModel.PANDA_OMRON, physics=True, seed=0)

    assert task.run()

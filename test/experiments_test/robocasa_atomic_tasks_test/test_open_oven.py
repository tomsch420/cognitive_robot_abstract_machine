"""
RoboCasa's OpenOven task performed by the PandaOmron in MuJoCo.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.doors import OpenOven
from experiments.robocasa_atomic_tasks.robots import RobotModel

# %% letting the oven door down


@pytest.mark.parametrize("seed", [0, 3])
def test_the_panda_omron_lets_the_oven_door_down_in_physics(seed: int):
    task = OpenOven(robot=RobotModel.PANDA_OMRON, physics=True, seed=seed)

    assert task.run()

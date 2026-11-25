"""
RoboCasa's PickPlaceCounterToOven task performed by the PandaOmron in MuJoCo.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.pick_place import PickPlaceCounterToOven
from experiments.robocasa_atomic_tasks.robots import RobotModel

# %% putting the object onto the oven tray


def test_the_panda_omron_puts_the_object_onto_the_oven_tray_in_physics():
    task = PickPlaceCounterToOven(robot=RobotModel.PANDA_OMRON, physics=True)

    assert task.run()

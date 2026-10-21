"""
Tiago carrying RoboCasa's tasks out in a RoboCasa kitchen, simulated in MuJoCo with
everything it moves held by the grip of its fingers alone.

These runs need RoboCasa's kitchen assets and a MuJoCo simulation, so they are skipped
wherever RoboCasa is not installed.
"""

import os

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.tiago_robocasa.drawers import CloseDrawer, OpenDrawer
from experiments.tiago_robocasa.pick_place import PickPlaceCounterToSink

runs_in_ci = pytest.mark.skipif(
    os.environ.get("CI", "false").lower() == "false",
    reason="MuJoCo tests only run in CI",
)


# %% the runs


@runs_in_ci
def test_tiago_puts_the_object_from_the_counter_into_the_sink():
    experiment = PickPlaceCounterToSink()
    experiment.build_world()

    experiment.run()

    assert experiment.task_is_done


@runs_in_ci
@pytest.mark.parametrize("task", [OpenDrawer, CloseDrawer])
def test_tiago_works_the_drawer(task):
    experiment = task()
    experiment.build_world()

    experiment.run()

    assert experiment.task_is_done

"""
Every one of RoboCasa's atomic tasks the PR2 performs.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.doors import ManipulateFixtureDoors
from experiments.robocasa_atomic_tasks.pick_place import OntoCounter, OntoReceptacle
from experiments.robocasa_atomic_tasks.toaster_oven import AdjustToasterOvenTemperature


def test_a_task_is_found_by_the_name_of_the_robocasa_task_it_performs():
    tasks = TaskModule.tasks_by_name

    assert (
        tasks[AdjustToasterOvenTemperature.robocasa_task_name]
        is AdjustToasterOvenTemperature
    )


def test_bases_that_perform_no_robocasa_task_of_their_own_are_left_out():
    tasks = set(TaskModule.tasks_by_name.values())

    assert not tasks & {ManipulateFixtureDoors, OntoCounter, OntoReceptacle}


def test_every_task_is_found_once():
    found = [task for module in TaskModule for task in module.tasks]

    assert len(found) == len(set(found))

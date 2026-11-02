"""
The actions the PR2 performs RoboCasa's atomic tasks with.
"""

from __future__ import annotations

import inspect
from abc import ABC

import pytest

from coraplex.robot_plans.actions.base import Action
from experiments.robocasa_atomic_tasks import skills


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

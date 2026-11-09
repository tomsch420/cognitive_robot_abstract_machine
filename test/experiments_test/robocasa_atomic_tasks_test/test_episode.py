"""
Tests of how an episode performs the robot's plans.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import TurnOnSinkFaucet

# %% executors in a physical simulation


@pytest.fixture(scope="module")
def physically_simulated_episode() -> Iterator[RoboCasaEpisode]:
    episode = TurnOnSinkFaucet(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield episode


@pytest.mark.parametrize("leaves_the_environment_to_the_physics", [True, False])
def test_an_executor_leaves_the_kitchen_to_the_physics_as_the_episode_says(
    physically_simulated_episode: RoboCasaEpisode,
    leaves_the_environment_to_the_physics: bool,
):
    physically_simulated_episode.leaves_the_environment_to_the_physics = (
        leaves_the_environment_to_the_physics
    )

    executor = physically_simulated_episode.create_executor()

    assert (
        executor.leaves_the_environment_to_the_physics
        == leaves_the_environment_to_the_physics
    )

"""
Tests of turning the water cold by pushing the faucet's lever with the base.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.robocasa_names import WaterTemperature
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import AdjustWaterTemperature
from .test_sink_actions import SinkOfAnEpisode

# %% the sink of a composed episode asking for cold water


@pytest.fixture(scope="module")
def composed_hot_sink() -> Iterator[SinkOfAnEpisode]:
    episode = AdjustWaterTemperature(robot=RobotModel("panda_omron"), seed=1).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def hot_sink(composed_hot_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    composed_hot_sink.episode.simulation.restore_snapshot(composed_hot_sink.composed)
    return composed_hot_sink


# %% turning the water cold


def test_the_episode_asks_for_cold_water(hot_sink: SinkOfAnEpisode):
    assert hot_sink.episode.environment.target_temp_state == WaterTemperature.COLD


def test_turning_the_water_cold_runs_the_water_cold(hot_sink: SinkOfAnEpisode):
    outcome = hot_sink.episode.run([hot_sink.turn_the_water_cold()])

    assert outcome.succeeded
    hot_sink.episode.judge.mirror.synchronize()
    assert hot_sink.episode.judge.pass_judgement().accomplished


def test_turning_the_water_cold_leaves_the_hand_outside_the_basin(
    hot_sink: SinkOfAnEpisode,
):
    hot_sink.episode.run([hot_sink.turn_the_water_cold()]).raise_if_failed()

    assert not hot_sink.hand_is_in_the_basin

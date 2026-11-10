"""
Tests of turning the water hot by pulling the faucet's lever with the base, and the
precondition that refuses it.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.exceptions import PreconditionsNotMet
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import AdjustWaterTemperature
from .test_sink_actions import SinkOfAnEpisode

# %% the sink of a composed episode asking for hot water


@pytest.fixture(scope="module")
def composed_cold_sink() -> Iterator[SinkOfAnEpisode]:
    episode = AdjustWaterTemperature(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def cold_sink(composed_cold_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    composed_cold_sink.episode.simulation.restore_snapshot(composed_cold_sink.composed)
    return composed_cold_sink


# %% turning the water hot


def test_turning_the_water_hot_swings_the_lever_to_its_hot_end(
    cold_sink: SinkOfAnEpisode,
):
    cold_sink.episode.run([cold_sink.lift_the_hand_out()]).raise_if_failed()
    action = cold_sink.turn_the_water_hot()

    outcome = cold_sink.episode.run([action])

    assert outcome.succeeded
    limits = cold_sink.temperature_joint.dof.limits
    assert (
        cold_sink.temperature_joint.position
        >= limits.upper.position - action.temperature_margin
    )


def test_turning_the_water_hot_leaves_the_hand_outside_the_basin(
    cold_sink: SinkOfAnEpisode,
):
    cold_sink.episode.run([cold_sink.lift_the_hand_out()]).raise_if_failed()

    cold_sink.episode.run([cold_sink.turn_the_water_hot()]).raise_if_failed()

    assert not cold_sink.hand_is_in_the_basin


def test_lifting_the_lever_after_turning_the_water_hot_runs_the_water_hot(
    cold_sink: SinkOfAnEpisode,
):
    cold_sink.episode.run([cold_sink.lift_the_hand_out()]).raise_if_failed()
    cold_sink.episode.run([cold_sink.turn_the_water_hot()]).raise_if_failed()

    outcome = cold_sink.episode.run([cold_sink.turn_on_the_faucet()])

    assert outcome.succeeded
    cold_sink.episode.judge.mirror.synchronize()
    assert cold_sink.episode.judge.pass_judgement().accomplished


def test_turning_the_water_hot_is_refused_while_the_hand_is_in_the_sink(
    cold_sink: SinkOfAnEpisode,
):
    outcome = cold_sink.episode.run([cold_sink.turn_the_water_hot()])

    assert isinstance(outcome.failure, PreconditionsNotMet)

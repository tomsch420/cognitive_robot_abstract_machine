"""
Tests of turning the faucet off with one finger, and the preconditions that refuse it.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.exceptions import (
    HandleHasNoBar,
    PreconditionsNotMet,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import TurnOffSinkFaucet
from experiments.robocasa_atomic_tasks.sink_actions import LeverBar
from .conftest import KitchenOfBoxes
from .test_sink_actions import SinkOfAnEpisode

# %% the lever's bar


def test_a_handle_without_a_cylinder_has_no_lever_bar(
    kitchen_of_boxes: KitchenOfBoxes,
):
    with pytest.raises(HandleHasNoBar):
        LeverBar.from_handle(kitchen_of_boxes.handle.root)


# %% the running sink of a composed episode


@pytest.fixture(scope="module")
def composed_running_sink() -> Iterator[SinkOfAnEpisode]:
    episode = TurnOffSinkFaucet(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def running_sink(composed_running_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    composed_running_sink.episode.simulation.restore_snapshot(
        composed_running_sink.composed
    )
    return composed_running_sink


# %% turning the faucet off


def test_turning_the_faucet_off_once_the_hand_is_out_stops_the_water(
    running_sink: SinkOfAnEpisode,
):
    running_sink.episode.run([running_sink.lift_the_hand_out()]).raise_if_failed()

    outcome = running_sink.episode.run([running_sink.turn_off_the_faucet()])

    assert outcome.succeeded
    running_sink.episode.judge.mirror.synchronize()
    assert running_sink.episode.judge.pass_judgement().accomplished


def test_turning_the_faucet_off_leaves_the_hand_outside_the_basin(
    running_sink: SinkOfAnEpisode,
):
    running_sink.episode.run([running_sink.lift_the_hand_out()]).raise_if_failed()

    running_sink.episode.run([running_sink.turn_off_the_faucet()]).raise_if_failed()

    assert not running_sink.hand_is_in_the_basin


def test_turning_the_faucet_off_is_refused_while_the_hand_is_in_the_sink(
    running_sink: SinkOfAnEpisode,
):
    outcome = running_sink.episode.run([running_sink.turn_off_the_faucet()])

    assert isinstance(outcome.failure, PreconditionsNotMet)


def test_turning_the_faucet_off_is_refused_once_the_water_stopped(
    running_sink: SinkOfAnEpisode,
):
    running_sink.episode.run([running_sink.lift_the_hand_out()]).raise_if_failed()
    running_sink.episode.run([running_sink.turn_off_the_faucet()]).raise_if_failed()

    outcome = running_sink.episode.run([running_sink.turn_off_the_faucet()])

    assert isinstance(outcome.failure, PreconditionsNotMet)

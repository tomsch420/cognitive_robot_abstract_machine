"""
Tests of turning the faucet's spout by pressing the side of the gripper against its bar.
"""

from __future__ import annotations

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.exceptions import PreconditionsNotMet
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.sink import TurnSinkSpout
from .test_sink_actions import SinkOfAnEpisode

# %% the sink of a composed episode asking for the spout turned


@pytest.fixture(scope="module")
def composed_spout_sink() -> Iterator[SinkOfAnEpisode]:
    episode = TurnSinkSpout(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def spout_sink(composed_spout_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    composed_spout_sink.episode.simulation.restore_snapshot(
        snapshot=composed_spout_sink.composed
    )
    return composed_spout_sink


def spout_goal(sink: SinkOfAnEpisode) -> float:
    """
    :param sink: The sink of an episode asking for the spout turned.
    :return: The position :class:`TurnSinkSpout` turns the spout to.
    """
    [goal] = TurnSinkSpout().joint_goals(sink.episode)
    return goal.world_position


# %% turning the spout


def test_turning_the_spout_turns_it_to_the_side_asked_for(
    spout_sink: SinkOfAnEpisode,
):
    spout_sink.episode.run([spout_sink.lift_the_hand_out()]).raise_if_failed()

    outcome = spout_sink.episode.run(
        [spout_sink.turn_the_spout(spout_goal(spout_sink))]
    )

    assert outcome.succeeded
    spout_sink.episode.judge.mirror.synchronize()
    assert spout_sink.episode.judge.pass_judgement().accomplished


def test_turning_the_spout_leaves_the_hand_outside_the_basin(
    spout_sink: SinkOfAnEpisode,
):
    spout_sink.episode.run([spout_sink.lift_the_hand_out()]).raise_if_failed()

    spout_sink.episode.run(
        [spout_sink.turn_the_spout(spout_goal(spout_sink))]
    ).raise_if_failed()

    assert not spout_sink.hand_is_in_the_basin


def test_turning_the_spout_is_refused_while_the_hand_is_in_the_sink(
    spout_sink: SinkOfAnEpisode,
):
    outcome = spout_sink.episode.run(
        [spout_sink.turn_the_spout(spout_goal(spout_sink))]
    )

    assert isinstance(outcome.failure, PreconditionsNotMet)

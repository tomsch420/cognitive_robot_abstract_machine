"""
Tests of putting an object into a fixture: taking it from above, carrying it with the
whole body to above a spot inside the fixture, and setting it down there.
"""

from __future__ import annotations

import math

import pytest
from typing_extensions import Iterator

pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_atomic_tasks.exceptions import NothingTouched
from experiments.robocasa_atomic_tasks.into_fixtures import (
    PutIntoAFixtureFromAboveAction,
)
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceCounterToSink
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import (
    LowerTheToolUntilTouchingAction,
    StandAction,
)
from krrood_spatial_math.spatial_types import Point3
from .test_sink_actions import SinkOfAnEpisode

# %% the sink of a composed episode asking for an object put into it


@pytest.fixture(scope="module")
def counter_to_sink() -> Iterator[SinkOfAnEpisode]:
    episode = PickPlaceCounterToSink(robot=RobotModel("panda_omron")).compose()
    with episode.executed(physics=True):
        yield SinkOfAnEpisode(
            episode=episode, composed=episode.simulation.take_snapshot()
        )


@pytest.fixture
def sink(counter_to_sink: SinkOfAnEpisode) -> SinkOfAnEpisode:
    counter_to_sink.episode.simulation.restore_snapshot(
        snapshot=counter_to_sink.composed
    )
    return counter_to_sink


def put_into_the_sink(sink: SinkOfAnEpisode) -> PutIntoAFixtureFromAboveAction:
    """
    :param sink: The sink of an episode asking for an object put into it.
    :return: A new action putting the object, taken by its first grasp from straight
        above, into the sink on the first spot the task offers.
    """
    episode = sink.episode
    task = PickPlaceCounterToSink()
    graspable = episode.graspable(task.picked_object)
    [grasp, *_] = episode.grasps.keep_the_grasps_that_fit(
        episode.grasps.from_above(graspable, pitches=(math.pi / 2,)),
        episode.arm.end_effector.separation_when_open,
    )
    [spot, *_] = task.spots(episode)
    return PutIntoAFixtureFromAboveAction(
        arm=episode.arm,
        grasp=grasp,
        spot=Point3(*spot, reference_frame=episode.world.root),
        fixture_bodies=episode.fixtures.bodies_of(task.fixture(episode)),
    )


def stand_facing_the_object(sink: SinkOfAnEpisode) -> StandAction:
    """
    :param sink: The sink of an episode asking for an object put into it.
    :return: A new step to the best stand facing the object.
    """
    episode = sink.episode
    position = episode.object_body(PickPlaceCounterToSink().picked_object)
    [stand, *_] = episode.stands.stands_facing(position.global_pose.to_np()[:3, 3])
    return StandAction(target_location=stand)


# %% putting the object into the sink


def test_putting_the_object_into_the_sink_from_above_accomplishes_the_task(
    sink: SinkOfAnEpisode,
):
    episode = sink.episode
    episode.run([sink.lift_the_hand_out()]).raise_if_failed()
    episode.run([stand_facing_the_object(sink)]).raise_if_failed()

    outcome = episode.run([put_into_the_sink(sink)])

    assert outcome.succeeded
    episode.judge.mirror.synchronize()
    assert episode.judge.pass_judgement().accomplished


# %% lowering the tool onto nothing


def test_lowering_the_tool_until_touching_fails_where_it_touches_nothing(
    sink: SinkOfAnEpisode,
):
    episode = sink.episode
    episode.run([sink.lift_the_hand_out()]).raise_if_failed()

    outcome = episode.run(
        [
            LowerTheToolUntilTouchingAction(
                arm=episode.arm,
                bodies=list(episode.arm.end_effector.bodies_with_collision),
                others=[sink.basin],
                distance=0.02,
            )
        ]
    )

    assert isinstance(outcome.failure, NothingTouched)

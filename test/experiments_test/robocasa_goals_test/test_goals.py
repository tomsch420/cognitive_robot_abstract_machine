"""
A RoboCasa task's goal: reached once every condition its success is stated by holds.
"""

from __future__ import annotations

from experiments.robocasa_goals.conditions import (
    ConditionObserver,
    GripperFarFromObjectDetector,
    ObjectInsideFixtureDetector,
)
from experiments.robocasa_goals.events import (
    LossOfRoboCasaConditionEvent,
    RoboCasaCondition,
    RoboCasaConditionChangeEvent,
    RoboCasaConditionEvent,
)
from experiments.robocasa_goals.goals import RoboCasaGoal
from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror
from experiments.robocasa_task_execution.counter_to_sink import CounterToSinkEpisode
from semantic_digital_twin.world_description.world_entity import Body

from .test_conditions import _drop_into_the_sink, _sink_of

# %% the goal of putting an object into the sink


def _sink_goal(
    episode: CounterToSinkEpisode,
    mirror: RoboCasaSimulationMirror,
    gripper_distance_threshold: float = 0.25,
) -> RoboCasaGoal:
    """
    :return: The goal of the counter to sink task: the target object lies inside the
        sink and no gripper is near it.
    """
    target = episode.target_object.root
    return RoboCasaGoal(
        subject=target,
        conditions=[
            ObjectInsideFixtureDetector(
                tracked_object=target,
                mirror=mirror,
                fixture=_sink_of(episode),
                partial_check=True,
            ),
            GripperFarFromObjectDetector(
                tracked_object=target, distance_threshold=gripper_distance_threshold
            ),
        ],
    )


def _task_success_events(
    events: list[RoboCasaConditionChangeEvent],
) -> list[tuple[type, Body]]:
    """
    :return: The kind and subject of every event about the task's success.
    """
    return [
        (type(event), event.tracked_object)
        for event in events
        if event.condition == RoboCasaCondition.TASK_SUCCEEDED
    ]


def test_a_goal_whose_conditions_all_hold_is_reached(episode, mirror):
    goal = _sink_goal(episode, mirror)
    observer = ConditionObserver(world=episode.task.world, detectors=goal.detectors())
    observer.observe()

    _drop_into_the_sink(episode)

    assert _task_success_events(observer.observe()) == [
        (RoboCasaConditionEvent, goal.subject)
    ]
    assert goal.reached


def test_a_goal_is_not_reached_while_one_of_its_conditions_does_not_hold(
    episode, mirror
):
    goal = _sink_goal(episode, mirror, gripper_distance_threshold=100.0)
    observer = ConditionObserver(world=episode.task.world, detectors=goal.detectors())
    _drop_into_the_sink(episode)

    assert _task_success_events(observer.observe()) == []
    assert not goal.reached


def test_a_goal_reached_is_lost_once_one_of_its_conditions_stops_holding(
    episode, mirror
):
    target = episode.target_object.root
    on_the_counter = target.parent_connection.origin
    goal = _sink_goal(episode, mirror)
    observer = ConditionObserver(world=episode.task.world, detectors=goal.detectors())
    _drop_into_the_sink(episode)
    observer.observe()

    target.parent_connection.origin = on_the_counter

    assert _task_success_events(observer.observe()) == [
        (LossOfRoboCasaConditionEvent, goal.subject)
    ]

"""
Detecting when the conditions RoboCasa's tasks state their success by start and stop
holding.
"""

from __future__ import annotations

import numpy as np

from experiments.robocasa_goals.conditions import (
    ConditionObserver,
    GripperFarFromObjectDetector,
    ObjectInReceptacleDetector,
    ObjectInsideFixtureDetector,
    ObjectTouchesAnyCounterDetector,
    ObjectTouchesFixtureDetector,
)
from experiments.robocasa_goals.events import (
    LossOfRoboCasaConditionEvent,
    RoboCasaCondition,
    RoboCasaConditionChangeEvent,
    RoboCasaConditionEvent,
)
from experiments.robocasa_task_execution.counter_to_sink import (
    CounterToSinkEpisode,
    RoboCasaObjectName,
)
from semantic_digital_twin.robots.robot_parts import EndEffector
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import Body

# %% moving the target object


def _sink_of(episode: CounterToSinkEpisode) -> Body:
    """
    :return: The body of the sink the task is about.
    """
    return episode.task.world.get_body_by_name(episode.environment.sink.root_body)


def _drop_into_the_sink(episode: CounterToSinkEpisode) -> None:
    """
    Let the target object fall into the sink from where the robot lets go of it.
    """
    episode.target_object.root.parent_connection.origin = (
        episode.placement_pose().homogeneous_matrix
    )
    episode.let_the_target_object_fall()


def _summary_of(
    events: list[RoboCasaConditionChangeEvent],
) -> list[tuple[type, RoboCasaCondition, Body, Body | None]]:
    """
    :return: What each event says: its kind, its condition and the bodies involved.
    """
    return [
        (type(event), event.condition, event.tracked_object, event.with_object)
        for event in events
    ]


# %% an object inside a fixture


def test_an_object_falling_into_the_sink_comes_to_lie_inside_it(episode, mirror):
    target, sink = episode.target_object.root, _sink_of(episode)
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[
            ObjectInsideFixtureDetector(
                tracked_object=target, mirror=mirror, fixture=sink, partial_check=True
            )
        ],
    )
    assert observer.observe() == []

    _drop_into_the_sink(episode)

    assert _summary_of(observer.observe()) == [
        (
            RoboCasaConditionEvent,
            RoboCasaCondition.OBJECT_INSIDE_FIXTURE,
            target,
            sink,
        )
    ]


def test_a_condition_that_still_holds_is_not_reported_again(episode, mirror):
    _drop_into_the_sink(episode)
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[
            ObjectInsideFixtureDetector(
                tracked_object=episode.target_object.root,
                mirror=mirror,
                fixture=_sink_of(episode),
                partial_check=True,
            )
        ],
    )
    observer.observe()

    assert observer.observe() == []


def test_an_object_taken_out_of_the_sink_is_no_longer_inside_it(episode, mirror):
    target, sink = episode.target_object.root, _sink_of(episode)
    on_the_counter = target.parent_connection.origin
    _drop_into_the_sink(episode)
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[
            ObjectInsideFixtureDetector(
                tracked_object=target, mirror=mirror, fixture=sink, partial_check=True
            )
        ],
    )
    observer.observe()

    target.parent_connection.origin = on_the_counter

    assert _summary_of(observer.observe()) == [
        (
            LossOfRoboCasaConditionEvent,
            RoboCasaCondition.OBJECT_INSIDE_FIXTURE,
            target,
            sink,
        )
    ]


# %% contacts


def test_an_object_resting_on_the_counter_touches_a_counter(episode, mirror):
    target = episode.target_object.root
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[
            ObjectTouchesAnyCounterDetector(tracked_object=target, mirror=mirror)
        ],
    )

    assert _summary_of(observer.observe()) == [
        (
            RoboCasaConditionEvent,
            RoboCasaCondition.OBJECT_TOUCHES_ANY_COUNTER,
            target,
            None,
        )
    ]


def test_an_object_lying_in_the_sink_touches_the_sink(episode, mirror):
    target, sink = episode.target_object.root, _sink_of(episode)
    _drop_into_the_sink(episode)
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[
            ObjectTouchesFixtureDetector(
                tracked_object=target, mirror=mirror, fixture=sink
            )
        ],
    )

    assert _summary_of(observer.observe()) == [
        (
            RoboCasaConditionEvent,
            RoboCasaCondition.OBJECT_TOUCHES_FIXTURE,
            target,
            sink,
        )
    ]


def test_an_object_put_where_another_one_stands_is_in_it(episode, mirror):
    world = episode.task.world
    target = episode.target_object.root
    receptacle = world.get_body_by_name(
        next(
            robocasa_object.root_body
            for name, robocasa_object in episode.environment.objects.items()
            if name != RoboCasaObjectName.TARGET
        )
    )
    target.parent_connection.origin = HomogeneousTransformationMatrix(
        receptacle.global_pose.to_np(), reference_frame=world.root
    )
    observer = ConditionObserver(
        world=world,
        detectors=[
            ObjectInReceptacleDetector(
                tracked_object=target, mirror=mirror, receptacle=receptacle
            )
        ],
    )

    assert _summary_of(observer.observe()) == [
        (
            RoboCasaConditionEvent,
            RoboCasaCondition.OBJECT_IN_RECEPTACLE,
            target,
            receptacle,
        )
    ]


# %% grippers


def _nearest_tool_frame_distance(episode: CounterToSinkEpisode) -> float:
    """
    :return: How far the target object stands from the nearest tool frame.
    """
    world = episode.task.world
    target_position = episode.target_object.root.global_pose.to_np()[:3, 3]
    return min(
        float(
            np.linalg.norm(
                end_effector.tool_frame.global_pose.to_np()[:3, 3] - target_position
            )
        )
        for end_effector in world.get_semantic_annotations_by_type(EndEffector)
    )


def test_an_object_out_of_every_grippers_reach_is_far_from_the_grippers(episode):
    target = episode.target_object.root
    observer = ConditionObserver(
        world=episode.task.world,
        detectors=[GripperFarFromObjectDetector(tracked_object=target)],
    )

    assert _summary_of(observer.observe()) == [
        (
            RoboCasaConditionEvent,
            RoboCasaCondition.GRIPPER_FAR_FROM_OBJECT,
            target,
            None,
        )
    ]


def test_an_object_in_a_gripper_is_no_longer_far_from_the_grippers(episode):
    world = episode.task.world
    target = episode.target_object.root
    observer = ConditionObserver(
        world=world, detectors=[GripperFarFromObjectDetector(tracked_object=target)]
    )
    observer.observe()
    tool_frame = world.get_semantic_annotations_by_type(EndEffector)[0].tool_frame

    target.parent_connection.origin = HomogeneousTransformationMatrix(
        tool_frame.global_pose.to_np(), reference_frame=world.root
    )

    assert _summary_of(observer.observe()) == [
        (
            LossOfRoboCasaConditionEvent,
            RoboCasaCondition.GRIPPER_FAR_FROM_OBJECT,
            target,
            None,
        )
    ]


def test_a_gripper_within_the_distance_threshold_is_not_far(episode):
    detector = GripperFarFromObjectDetector(
        tracked_object=episode.target_object.root,
        distance_threshold=_nearest_tool_frame_distance(episode) + 0.01,
    )
    observer = ConditionObserver(world=episode.task.world, detectors=[detector])

    assert observer.observe() == []
    assert detector.holds is False

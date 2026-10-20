"""
Segmind detectors for the conditions RoboCasa's tasks state their success by.

A condition about objects and fixtures is judged by RoboCasa's own check, in RoboCasa's
simulation brought into the state of the world. A condition about the robot is judged in
the world, since RoboCasa's checks read RoboCasa's own robot rather than the one acting.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

from giskardpy.motion_statechart.context import MotionStatechartContext
from krrood.ormatic.utils import classproperty
from segmind.datastructures.events import DetectionEvent
from segmind.detectors.base import AbstractDetector, SegmindContext
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.robots.robot_parts import EndEffector
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

with robocasa_version_assertions_relaxed():
    from robocasa.utils import object_utils

from experiments.robocasa_goals.events import (
    LossOfRoboCasaConditionEvent,
    RoboCasaCondition,
    RoboCasaConditionChangeEvent,
    RoboCasaConditionEvent,
)
from experiments.robocasa_goals.simulation_mirror import RoboCasaSimulationMirror

# %% detecting a condition


@dataclass(repr=False, eq=False)
class RoboCasaConditionDetector(AbstractDetector, ABC):
    """
    Detects one of RoboCasa's conditions starting and stopping to hold for the tracked
    object.

    A condition that does not hold when first judged is not reported.
    """

    holds: bool | None = field(default=None, init=False)
    """
    Whether the condition held when last judged; ``None`` before it is judged.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        """
        The condition detected.
        """
        raise NotImplementedError

    @property
    def with_object(self) -> Body | None:
        """
        The other body the condition is about besides the tracked object, if any.
        """
        return None

    @abstractmethod
    def holds_now(self, context: MotionStatechartContext) -> bool:
        """
        :param context: The context of the world the condition is judged in.
        :return: Whether the condition holds in the current state of the world.
        """

    def update_context_and_events(
        self,
        context: MotionStatechartContext,
        segmind_context: SegmindContext,
        objects_to_check: list[Body],
    ) -> list[DetectionEvent]:
        """
        :return: The change of the condition since it was last judged, if it changed.
        """
        holds = self.holds_now(context)
        changed = holds != bool(self.holds)
        self.holds = holds
        if not changed:
            return []
        event_type = RoboCasaConditionEvent if holds else LossOfRoboCasaConditionEvent
        return [
            event_type(
                tracked_object=self.tracked_object,
                with_object=self.with_object,
                condition=self.condition,
            )
        ]


@dataclass(repr=False, eq=False)
class RoboCasaSimulationConditionDetector(RoboCasaConditionDetector, ABC):
    """
    Detects a condition RoboCasa checks in its own simulation.
    """

    mirror: RoboCasaSimulationMirror = field(kw_only=True)
    """
    Brings RoboCasa's simulation into the state of the world before the check.
    """

    @property
    def object_name(self) -> str:
        """
        The name RoboCasa gives the tracked object.
        """
        return self.mirror.object_name_of(self.tracked_object)

    def holds_now(self, context: MotionStatechartContext) -> bool:
        self.mirror.synchronize()
        return bool(self.check_in_robocasa())

    @abstractmethod
    def check_in_robocasa(self) -> bool:
        """
        :return: What RoboCasa's check says about its simulation as it stands.
        """


# %% conditions about objects and fixtures


@dataclass(repr=False, eq=False)
class ObjectInsideFixtureDetector(RoboCasaSimulationConditionDetector):
    """
    Detects the tracked object coming to lie inside a fixture and leaving it.
    """

    fixture: Body = field(kw_only=True)
    """
    The root body of the fixture.
    """

    partial_check: bool = field(default=False, kw_only=True)
    """
    Whether the origin of the object suffices to be inside, rather than its whole
    bounding box.
    """

    margin: float = field(default=0.05, kw_only=True)
    """
    How far the bounding box of the object may reach out of the fixture's interior; not
    used by a partial check.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.OBJECT_INSIDE_FIXTURE

    @property
    def with_object(self) -> Body | None:
        return self.fixture

    def check_in_robocasa(self) -> bool:
        return object_utils.obj_inside_of(
            self.mirror.environment,
            self.object_name,
            self.mirror.fixture_of(self.fixture),
            partial_check=self.partial_check,
            th=self.margin,
        )


@dataclass(repr=False, eq=False)
class ObjectInReceptacleDetector(RoboCasaSimulationConditionDetector):
    """
    Detects the tracked object coming to stand in another object and leaving it.
    """

    receptacle: Body = field(kw_only=True)
    """
    The root body of the object the tracked object stands in.
    """

    horizontal_distance_threshold: float | None = field(default=None, kw_only=True)
    """
    How far apart the two objects may stand horizontally; ``None`` for RoboCasa's
    default, which scales with the receptacle's horizontal radius.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.OBJECT_IN_RECEPTACLE

    @property
    def with_object(self) -> Body | None:
        return self.receptacle

    def check_in_robocasa(self) -> bool:
        return object_utils.check_obj_in_receptacle(
            self.mirror.environment,
            self.object_name,
            self.mirror.object_name_of(self.receptacle),
            th=self.horizontal_distance_threshold,
        )


@dataclass(repr=False, eq=False)
class ObjectTouchesFixtureDetector(RoboCasaSimulationConditionDetector):
    """
    Detects the tracked object starting and stopping to touch a fixture.
    """

    fixture: Body = field(kw_only=True)
    """
    The root body of the fixture.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.OBJECT_TOUCHES_FIXTURE

    @property
    def with_object(self) -> Body | None:
        return self.fixture

    def check_in_robocasa(self) -> bool:
        return object_utils.check_obj_fixture_contact(
            self.mirror.environment,
            self.object_name,
            self.mirror.fixture_of(self.fixture),
        )


@dataclass(repr=False, eq=False)
class ObjectTouchesAnyCounterDetector(RoboCasaSimulationConditionDetector):
    """
    Detects the tracked object starting and stopping to touch any of the kitchen's
    counters.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.OBJECT_TOUCHES_ANY_COUNTER

    def check_in_robocasa(self) -> bool:
        return object_utils.check_obj_any_counter_contact(
            self.mirror.environment, self.object_name
        )


# %% conditions about the robot


@dataclass(repr=False, eq=False)
class GripperFarFromObjectDetector(RoboCasaConditionDetector):
    """
    Detects every gripper of the world getting far from the tracked object, and one of
    them coming near it again.
    """

    distance_threshold: float = field(default=0.25, kw_only=True)
    """
    How far the tool frame of every gripper stands from the origin of the object at
    least to be far from it.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.GRIPPER_FAR_FROM_OBJECT

    def holds_now(self, context: MotionStatechartContext) -> bool:
        object_position = self.tracked_object.global_pose.to_np()[:3, 3]
        return all(
            np.linalg.norm(
                end_effector.tool_frame.global_pose.to_np()[:3, 3] - object_position
            )
            > self.distance_threshold
            for end_effector in context.world.get_semantic_annotations_by_type(
                EndEffector
            )
        )


# %% judging conditions on demand


@dataclass
class ConditionObserver:
    """
    Judges detectors in a world whenever asked, logging what they detect as segmind
    events.

    .. note::
        The detectors are judged one after another in their order, without a motion
        statechart, so that a world whose state is rebuilt as objects are attached to
        the robot is still observed.
    """

    world: World
    """
    The world the detectors judge.
    """

    detectors: list[AbstractDetector]
    """
    The detectors, each after those it reads from.
    """

    context: MotionStatechartContext = field(init=False)
    """
    The context the detectors are judged in, holding the log of their events.
    """

    def __post_init__(self) -> None:
        """
        Create the context the detectors are judged in.
        """
        self.context = MotionStatechartContext(world=self.world)
        self.context.add_extension(SegmindContext())

    def observe(self) -> list[DetectionEvent]:
        """
        Judge every detector in the current state of the world.

        :return: The events detected since the last observation, in the order they were
            logged.
        """
        for detector in self.detectors:
            detector.on_tick(self.context)
        logger = self.context.require_extension(SegmindContext).logger
        return list(iter(logger.get_next_event, None))

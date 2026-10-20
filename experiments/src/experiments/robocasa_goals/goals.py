"""
The goal of a RoboCasa task: reached once every condition the task states its success by
holds.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from giskardpy.motion_statechart.context import MotionStatechartContext
from krrood.ormatic.utils import classproperty
from segmind.detectors.base import AbstractDetector
from semantic_digital_twin.world_description.world_entity import Body

from experiments.robocasa_goals.conditions import RoboCasaConditionDetector
from experiments.robocasa_goals.events import RoboCasaCondition

# %% the goal


@dataclass
class RoboCasaGoal:
    """
    The goal of a RoboCasa task, as the conditions that all hold once it succeeded.
    """

    subject: Body
    """
    The object the task is about.
    """

    conditions: list[RoboCasaConditionDetector]
    """
    The detectors of the conditions.
    """

    @property
    def reached(self) -> bool:
        """
        Whether every condition held when last judged.
        """
        return all(condition.holds for condition in self.conditions)

    def detectors(self) -> list[AbstractDetector]:
        """
        :return: The detectors of the conditions, followed by the detector of the goal
            itself, which reads them.
        """
        return [
            *self.conditions,
            TaskSucceededDetector(tracked_object=self.subject, goal=self),
        ]


# %% detecting the goal


@dataclass(repr=False, eq=False)
class TaskSucceededDetector(RoboCasaConditionDetector):
    """
    Detects a goal being reached and lost again, judged after its conditions.
    """

    goal: RoboCasaGoal = field(kw_only=True)
    """
    The goal detected.
    """

    @classproperty
    def condition(cls) -> RoboCasaCondition:
        return RoboCasaCondition.TASK_SUCCEEDED

    def holds_now(self, context: MotionStatechartContext) -> bool:
        return self.goal.reached

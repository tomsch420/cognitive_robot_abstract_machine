"""
The segmind events reporting that one of the conditions RoboCasa's tasks state their
success by starts or stops holding.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import StrEnum

from krrood.ormatic.utils import classproperty
from segmind.datastructures.events import EventWithTrackedObjects

# %% the conditions


class RoboCasaCondition(StrEnum):
    """
    The conditions RoboCasa's tasks state their success by, each valued by the name of
    the RoboCasa function that checks it.
    """

    OBJECT_INSIDE_FIXTURE = "obj_inside_of"
    """
    An object lies inside a fixture, such as a sink or a cabinet.
    """

    OBJECT_IN_RECEPTACLE = "check_obj_in_receptacle"
    """
    An object stands in another object, such as a bowl or a plate, touching it.
    """

    OBJECT_TOUCHES_FIXTURE = "check_obj_fixture_contact"
    """
    An object touches a fixture.
    """

    OBJECT_TOUCHES_ANY_COUNTER = "check_obj_any_counter_contact"
    """
    An object touches one of the kitchen's counters.
    """

    GRIPPER_FAR_FROM_OBJECT = "gripper_obj_far"
    """
    No gripper is near an object.
    """

    TASK_SUCCEEDED = "_check_success"
    """
    Every condition a task states its success by holds.
    """


# %% the events


@dataclass(unsafe_hash=True, kw_only=True)
class RoboCasaConditionChangeEvent(EventWithTrackedObjects, ABC):
    """
    A condition starts or stops holding for the tracked object, and with the other
    object it is about, if any.
    """

    condition: RoboCasaCondition = field(kw_only=True)
    """
    The condition that changed.
    """

    @classproperty
    def condition_holds(cls) -> bool:
        """
        Whether the condition holds after the change.
        """
        raise NotImplementedError

    def __str__(self) -> str:
        names = " - ".join(str(body.name) for body in self.tracked_objects)
        return f"{self.__class__.__name__} {self.condition}: {names} - {self.timestamp}"


@dataclass(unsafe_hash=True, kw_only=True)
class RoboCasaConditionEvent(RoboCasaConditionChangeEvent):
    """
    A condition starts holding.
    """

    @classproperty
    def condition_holds(cls) -> bool:
        return True


@dataclass(unsafe_hash=True, kw_only=True)
class LossOfRoboCasaConditionEvent(RoboCasaConditionChangeEvent):
    """
    A condition stops holding.
    """

    @classproperty
    def condition_holds(cls) -> bool:
        return False

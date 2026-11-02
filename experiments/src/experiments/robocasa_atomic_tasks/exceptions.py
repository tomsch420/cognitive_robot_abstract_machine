from __future__ import annotations

from dataclasses import dataclass

from coraplex.plans.failures import PlanFailure
from giskardpy.qp.exceptions import InfeasibleException
from krrood.exceptions import DataclassException
from semantic_digital_twin.world_description.world_entity import Body

# %% placing


@dataclass
class NoPlacementSpot(DataclassException):
    """
    Raised when an object is to be put down but no spot is found that it fits on.
    """

    body: Body
    """
    The object to be put down.
    """

    def error_message(self) -> str:
        return f"No spot was found to put {self.body.name} down on."

    def suggest_correction(self) -> str:
        return "Clear the place it is to go to, or compose another episode."


# %% picking up


@dataclass
class ObjectNotPickedUp(DataclassException):
    """
    Raised when every pick-up of an object in a physical simulation left it behind.
    """

    body: Body
    """
    The object to be picked up.
    """

    def error_message(self) -> str:
        return f"The gripper did not hold {self.body.name} after picking it up."

    def suggest_correction(self) -> str:
        return "Try other grasps, or a firmer grip."


# %% moving


@dataclass
class MotionWasInfeasible(PlanFailure):
    """
    Raised when the motion controller found no motion that satisfies every constraint at
    once, such as when the robot starts a motion closer to something than collision
    avoidance allows.

    Like coraplex's :class:`~coraplex.plans.failures.MotionMadeNoProgress`, this says
    that the attempt did not work from where it started rather than that the plan
    cannot go on, so that another candidate is tried.
    """

    infeasibility: InfeasibleException
    """
    The infeasibility the solver reported.
    """

    def error_message(self) -> str:
        return self.infeasibility.error_message()

    def suggest_correction(self) -> str:
        return "Start the motion clear of the kitchen, or try another candidate."

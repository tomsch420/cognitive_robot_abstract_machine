from __future__ import annotations

from dataclasses import dataclass

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


# %% fixtures


@dataclass
class JointNotPushedToItsEnd(DataclassException):
    """
    Raised when every push of a part left the joint it hangs from short of the end of
    its travel.
    """

    joint_name: str
    """
    The name RoboCasa gives the joint.
    """

    position: float
    """
    Where the joint stands after the last push.
    """

    end: float
    """
    The end of its travel it was pushed towards.
    """

    def error_message(self) -> str:
        return (
            f"{self.joint_name} stands at {self.position:.4f}, not at {self.end:.4f}."
        )

    def suggest_correction(self) -> str:
        return "Push the part from other sides, or check what holds it back."


@dataclass
class NoRackAtLevel(DataclassException):
    """
    Raised when an oven has neither a rack nor a tray at the level asked for, nor at the
    bottom.
    """

    fixture_name: str
    """
    The name RoboCasa gives the oven.
    """

    level: int
    """
    The level asked for, 0 at the bottom.
    """

    def error_message(self) -> str:
        return f"{self.fixture_name} has no rack or tray at level {self.level}."

    def suggest_correction(self) -> str:
        return "Ask for a level the oven has a rack or a tray at."


# %% driving


@dataclass
class NoPathThroughTheKitchen(DataclassException):
    """
    Raised when the free space of the kitchen's floor connects no spot near where the
    robot stands with one near where it is to stand, however little clearance it keeps.
    """

    smallest_clearance: float
    """
    The least clearance from the kitchen, in metres, a path was searched for with.
    """

    def error_message(self) -> str:
        return (
            f"No path through the kitchen's free space keeps "
            f"{self.smallest_clearance:.2f} m clear of the kitchen."
        )

    def suggest_correction(self) -> str:
        return "Compose another episode, or let the robot keep less clearance."

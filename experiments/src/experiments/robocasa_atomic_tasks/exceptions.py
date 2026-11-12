from __future__ import annotations

from abc import ABC
from dataclasses import dataclass

from coraplex.plans.failures import PlanFailure
from krrood.exceptions import DataclassException
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
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


# %% stored runs


@dataclass
class NoStoredRun(DataclassException):
    """
    Raised when a run of a task is to be replayed but none is stored.
    """

    task: str
    """
    The name of the task.
    """

    def error_message(self) -> str:
        return f"No run of {self.task} is stored."

    def suggest_correction(self) -> str:
        return "Run the task with physics and a database to store its run in first."


@dataclass
class DegreesOfFreedomNotInTheWorld(DataclassException):
    """
    Raised when a stored run is to be replayed in a world that lacks degrees of freedom
    the run moved.
    """

    names: list[str]
    """
    The names of the degrees of freedom the world lacks.
    """

    def error_message(self) -> str:
        return f"The world has no degrees of freedom named {', '.join(self.names)}."

    def suggest_correction(self) -> str:
        return (
            "Compose the episode with the layout, style, seed and robot the run was "
            "recorded with."
        )


# %% actions at the sink


@dataclass
class PreconditionsNotMet(PlanFailure):
    """
    Raised when an action starts in a world its preconditions do not hold in.
    """

    action_name: str
    """
    The name of the action.
    """

    false_preconditions: list[str]
    """
    The preconditions that do not hold, in words.
    """

    def error_message(self) -> str:
        return (
            f"{self.action_name} cannot start: {'; '.join(self.false_preconditions)}."
        )

    def suggest_correction(self) -> str:
        return "Bring the world into a state these hold in first, by another action."


@dataclass
class FaucetHandleStoppedShort(PlanFailure, ABC):
    """
    Raised when a push left the faucet's handle short of where it was to be pushed.
    """

    lift_joint: ActiveConnection1DOF
    """
    The joint the handle is lifted about.
    """

    position: float
    """
    Where the handle stands after the push, in radians.
    """

    goal: float
    """
    Where the handle was to be pushed to, in radians.
    """

    def error_message(self) -> str:
        return (
            f"{self.lift_joint.name} stands at {self.position:.3f} rad after the push, "
            f"short of {self.goal:.3f} rad."
        )

    def suggest_correction(self) -> str:
        return "Push further, or put the finger closer to the handle."


@dataclass
class FaucetNotTurnedOn(FaucetHandleStoppedShort):
    """
    Raised when a push left the faucet's handle short of where the water runs.
    """


@dataclass
class FaucetNotTurnedOff(FaucetHandleStoppedShort):
    """
    Raised when a push left the faucet's handle above where the water stops.
    """


@dataclass
class HandleHasNoBar(DataclassException):
    """
    Raised when a faucet's handle has no cylinder in its collision to push as its bar.
    """

    handle: Body
    """
    The handle.
    """

    def error_message(self) -> str:
        return f"{self.handle.name} has no cylinder in its collision."

    def suggest_correction(self) -> str:
        return "Push a lever handle, whose bar is a cylinder."


@dataclass
class LeverSwungAside(PlanFailure):
    """
    Raised when turning the faucet on swung its handle about another of its joints, such
    as the one choosing the water's temperature.
    """

    joint: ActiveConnection1DOF
    """
    The joint the handle was swung about.
    """

    change: float
    """
    How far the handle was swung about it, in radians.
    """

    tolerance: float
    """
    How far it may be swung, in radians.
    """

    def error_message(self) -> str:
        return (
            f"{self.joint.name} moved by {self.change:.3f} rad, more than "
            f"{self.tolerance:.3f} rad."
        )

    def suggest_correction(self) -> str:
        return "Bring the finger in beside the handle rather than against it."


@dataclass
class LeverNotSwungFarEnough(PlanFailure):
    """
    Raised when swinging a faucet's lever about its temperature joint left it short of
    the position it was swung towards.
    """

    temperature_joint: ActiveConnection1DOF
    """
    The joint the lever is swung about to choose the water's temperature.
    """

    position: float
    """
    Where the lever stands after the swing, in radians.
    """

    goal: float
    """
    Where the lever was to be swung to, in radians.
    """

    def error_message(self) -> str:
        return (
            f"{self.temperature_joint.name} stands at {self.position:.3f} rad after "
            f"the swing, short of {self.goal:.3f} rad."
        )

    def suggest_correction(self) -> str:
        return "Swing further, or bring the finger closer to the lever."


@dataclass
class SpoutHasNoBar(DataclassException):
    """
    Raised when a faucet's spout has no level cylinder in its collision to push as its
    bar.
    """

    spout: Body
    """
    The spout.
    """

    def error_message(self) -> str:
        return f"{self.spout.name} has no level cylinder in its collision."

    def suggest_correction(self) -> str:
        return "Turn a spout whose bar is a level cylinder."


@dataclass
class SpoutNotTurned(PlanFailure):
    """
    Raised when turning a faucet's spout left it short of the position it was turned to.
    """

    spout_joint: ActiveConnection1DOF
    """
    The joint the spout turns about.
    """

    position: float
    """
    Where the spout stands after the turn, in radians.
    """

    goal: float
    """
    Where the spout was to be turned to, in radians.
    """

    def error_message(self) -> str:
        return (
            f"{self.spout_joint.name} stands at {self.position:.3f} rad after the "
            f"turn, short of {self.goal:.3f} rad."
        )

    def suggest_correction(self) -> str:
        return "Press the gripper harder against the bar, or turn further."


# %% touching


@dataclass
class NothingTouched(PlanFailure):
    """
    Raised when the tool frame went its whole way down without a body of one group
    coming to touch a body of the other.
    """

    body_names: list[str]
    """
    The names of the bodies that were to touch.
    """

    other_names: list[str]
    """
    The names of the bodies they were to touch.
    """

    distance: float
    """
    How far the tool frame went down, in metres.
    """

    def error_message(self) -> str:
        return (
            f"None of {', '.join(self.body_names)} touched any of "
            f"{', '.join(self.other_names)} within {self.distance:.3f} m."
        )

    def suggest_correction(self) -> str:
        return (
            "Start lower or above the bodies to touch, or let the tool frame go further "
            "down."
        )

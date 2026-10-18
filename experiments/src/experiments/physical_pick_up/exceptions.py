from __future__ import annotations

from dataclasses import dataclass
from krrood.exceptions import DataclassException

# %% grasping


@dataclass
class NoDefaultGraspError(DataclassException):
    """
    Raised when an attempt is to take an object by its default grasp, but no grasp is
    known to lift the object.
    """

    body_name: str
    """
    The name of the object's body.
    """

    def error_message(self) -> str:
        return (
            f"No grasp was given for '{self.body_name}', and its description knows "
            f"none that lifts it."
        )

    def suggest_correction(self) -> str:
        return "Give the attempt a grasp, for instance one drawn from the object's statement."

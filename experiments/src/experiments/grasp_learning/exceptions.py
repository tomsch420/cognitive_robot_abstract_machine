"""
The exceptions of grasp learning.
"""

from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException

# %% the kitchen around an object


@dataclass(kw_only=True)
class ObjectInNoFixture(DataclassException):
    """
    Raised when a kitchen is to be cut down around the fixture an object lies in, but no
    fixture's bounding box holds the middle of the object.
    """

    object_name: str
    """
    The name RoboCasa gives the object.
    """

    def error_message(self) -> str:
        return f"No fixture of the kitchen holds the {self.object_name}."

    def suggest_correction(self) -> str:
        return "Choose a task whose object starts inside a fixture, such as a sink."


# %% stored attempts


@dataclass(kw_only=True)
class NoRecordedGraspAttempt(DataclassException):
    """
    Raised when an attempt with its run is read by a number no such attempt is stored
    under.
    """

    database_id: int
    """
    The number asked for.
    """

    def error_message(self) -> str:
        return f"No attempt with its run is stored under {self.database_id}."

    def suggest_correction(self) -> str:
        return "Read the numbers of the stored attempts from the database first."

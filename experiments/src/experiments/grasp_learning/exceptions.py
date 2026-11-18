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


@dataclass(kw_only=True)
class NoStoredWorld(DataclassException):
    """
    Raised when a world is read by a number no world is stored under.
    """

    database_id: int
    """
    The number asked for.
    """

    def error_message(self) -> str:
        return f"No world is stored under {self.database_id}."

    def suggest_correction(self) -> str:
        return "Read the numbers of the stored worlds from the database first."


@dataclass(kw_only=True)
class NoPickUpTrial(DataclassException):
    """
    Raised when a pick-up trial is read by a number no trial is stored under.
    """

    database_id: int
    """
    The number asked for.
    """

    def error_message(self) -> str:
        return f"No pick-up trial is stored under {self.database_id}."

    def suggest_correction(self) -> str:
        return "Read the numbers of the stored trials from the database first."


# %% the starting world of pick-ups


@dataclass(kw_only=True)
class StartingWorldDoesNotMatch(DataclassException):
    """
    Raised when the state of a stored world is put into a world whose degrees of freedom
    have other names.
    """

    world_id: int
    """
    The database id of the stored world.
    """

    missing: list[str]
    """
    The names of the stored world's degrees of freedom the other world lacks.
    """

    extra: list[str]
    """
    The names of the other world's degrees of freedom the stored world lacks.
    """

    def error_message(self) -> str:
        return (
            f"The world stored under {self.world_id} does not match: the world lacks "
            f"{self.missing} and has {self.extra} besides."
        )

    def suggest_correction(self) -> str:
        return (
            "Compose the episode the stored world was saved from, with the same task, "
            "seed and cut."
        )

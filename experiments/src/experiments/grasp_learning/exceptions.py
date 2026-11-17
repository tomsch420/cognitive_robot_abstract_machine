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

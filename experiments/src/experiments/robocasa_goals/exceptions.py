from __future__ import annotations

from dataclasses import dataclass

from krrood.exceptions import DataclassException
from semantic_digital_twin.world_description.world_entity import Body

# %% finding RoboCasa's counterpart of a body


@dataclass
class NotARoboCasaObject(DataclassException):
    """
    Raised when a body is asked for the RoboCasa object it is the root of, but is the
    root of none.
    """

    body: Body
    """
    The body asked about.
    """

    def error_message(self) -> str:
        return f"{self.body.name} is the root body of no object RoboCasa placed."

    def suggest_correction(self) -> str:
        return "Ask about the root body of one of the task's objects."


@dataclass
class NotARoboCasaFixture(DataclassException):
    """
    Raised when a body is asked for the RoboCasa fixture it is the root of, but is the
    root of none.
    """

    body: Body
    """
    The body asked about.
    """

    def error_message(self) -> str:
        return f"{self.body.name} is the root body of no fixture of the kitchen."

    def suggest_correction(self) -> str:
        return "Ask about the root body of one of the kitchen's fixtures."

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from typing_extensions import Type

from krrood.exceptions import DataclassException
from semantic_digital_twin.semantic_annotations.mixins import PartWholeRelationship

# %% stored parts


@dataclass
class NoStoredPartsError(DataclassException):
    """
    Raised when the parts of a mesh are asked for that no stored answer was located in.
    """

    whole_type: Type[PartWholeRelationship]
    """
    The type of the annotation whose parts were asked for.
    """

    directory: Path
    """
    Where the stored answers are.
    """

    def error_message(self) -> str:
        return (
            f"No answer stored in {self.directory} was located in the mesh of the "
            f"{self.whole_type.__name__} asked about."
        )

    def suggest_correction(self) -> str:
        return (
            "Locate the parts of the object with the part finding server and store the "
            "answer together with the mesh it was located in."
        )

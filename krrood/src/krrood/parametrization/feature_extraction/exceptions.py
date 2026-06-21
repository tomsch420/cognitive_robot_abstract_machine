from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import Type

from krrood.exceptions import DataclassException


@dataclass
class UnsupportedFeatureTypeError(DataclassException, TypeError):
    """
    Raised when a feature column has a type that cannot be preprocessed for JPT training.
    """

    feature_type: Type
    """
    The Python type that is not supported as a JPT feature column.
    """

    column_name: str
    """
    The DataFrame column name whose type is unsupported.
    """

    def error_message(self) -> str:
        return (
            f"Feature column '{self.column_name}' has unsupported type "
            f"'{self.feature_type.__name__}' for JPT preprocessing."
        )

    def suggest_correction(self) -> str:
        return (
            "Ensure all feature columns hold bool, int, float, or Enum values. "
            "Add an explicit preprocessing step for custom types."
        )

from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import Type

from krrood.exceptions import DataclassException
from random_events.variable import compatible_types


@dataclass
class UnsupportedFeatureTypeError(DataclassException, TypeError):
    """
    Raised when a feature column has a type that cannot be preprocessed for probabilistic learning.
    """

    feature_type: Type
    """
    The Python type that is not supported as a feature column.
    """

    column_name: str
    """
    The DataFrame column name whose type is unsupported.
    """

    def error_message(self) -> str:
        return (
            f"Feature column '{self.column_name}' has unsupported type "
            f"'{self.feature_type.__name__}' for probabilistic learning."
        )

    def suggest_correction(self) -> str:
        supported = ", ".join(t.__name__ for t in compatible_types)
        return (
            f"Ensure all feature columns hold values of a compatible type: {supported}. "
            "Add an explicit preprocessing step for custom types."
        )

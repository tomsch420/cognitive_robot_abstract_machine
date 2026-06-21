from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Type

from krrood.exceptions import DataclassException


@dataclass
class CircuitNotFittedError(DataclassException):
    """
    Raised when a RelationalProbabilisticCircuit is grounded before it has been fitted.
    """

    class_: Type
    """
    The domain class whose relational circuit has not been fitted yet.
    """

    def error_message(self) -> str:
        return (
            f"RelationalProbabilisticCircuit for {self.class_.__name__} must be fitted "
            f"before it can be grounded."
        )

    def suggest_correction(self) -> str:
        return "Call `fit` with training instances before calling `ground`."


@dataclass
class GroundingFailedError(DataclassException):
    """
    Raised when grounding a circuit produces an empty or degenerate result.
    """

    context: str
    """
    A description of which grounding step failed (e.g. part index or class name).
    """

    def error_message(self) -> str:
        return f"Grounding failed for: {self.context}."

    def suggest_correction(self) -> str:
        return (
            "Ensure the query is compatible with the fitted circuit and that "
            "the conditioning event has non-zero probability."
        )

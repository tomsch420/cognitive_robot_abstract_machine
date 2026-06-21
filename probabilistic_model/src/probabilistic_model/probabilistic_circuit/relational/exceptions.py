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

    Carries the class name and the grounding step that failed so that stack traces
    immediately show which part of the relational query could not be resolved.
    """

    class_name: str
    """
    Name of the domain class whose circuit was being grounded.
    """

    step: str
    """
    Human-readable description of the grounding step that produced an empty circuit
    (e.g. ``"class circuit"`` or ``"exchangeable part at index 2"``).
    """

    def error_message(self) -> str:
        return (
            f"Grounding '{self.class_name}' failed at step '{self.step}': "
            f"the resulting circuit is empty."
        )

    def suggest_correction(self) -> str:
        return (
            "Ensure the query structure is compatible with the fitted circuit and that "
            "every conditioning event has non-zero probability under the training distribution."
        )

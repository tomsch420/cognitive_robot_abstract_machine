"""
Exceptions of learning the parameters and the structure of layered circuits.
"""

from __future__ import annotations

from dataclasses import dataclass

from random_events.variable import Variable

from probabilistic_model.exceptions import ProbabilisticModelException

# %% exceptions


@dataclass(kw_only=True)
class UndifferentiableBackendError(ProbabilisticModelException, TypeError):
    """
    Exception raised when the parameters of a circuit are arrays of a backend that
    cannot differentiate, so expectation maximization cannot count its statistics.
    """

    parameter_type: type
    """
    The type of the parameter arrays.
    """

    def error_message(self) -> str:
        return (
            f"The parameters are of type {self.parameter_type.__name__}, whose backend "
            f"does not differentiate."
        )

    def suggest_correction(self) -> str:
        return (
            "Store the parameters in jax or torch with "
            "LayeredProbabilisticCircuit.to_namespace."
        )


@dataclass(kw_only=True)
class UnsupportedEntropyLayerError(ProbabilisticModelException, TypeError):
    """
    Exception raised when the entropy of the nodes of a layer is not known in closed
    form.
    """

    layer_type: type
    """
    The type of the layer.
    """

    def error_message(self) -> str:
        return f"The entropy of a {self.layer_type.__name__} is not known."

    def suggest_correction(self) -> str:
        return "Use discrete or Gaussian input layers."


@dataclass(kw_only=True)
class UnsupportedStructureVariableError(ProbabilisticModelException, TypeError):
    """
    Exception raised when a structure learner meets a variable it has no input
    distribution for.
    """

    variable: Variable
    """
    The variable.
    """

    def error_message(self) -> str:
        return (
            f"The variable {self.variable.name} of type {type(self.variable).__name__} "
            f"has no input distribution in a learned structure."
        )

    def suggest_correction(self) -> str:
        return "Use symbolic or continuous variables."

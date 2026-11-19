"""
Exceptions of training and querying the circuits of the benchmark with cirkit.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

from krrood.exceptions import DataclassException
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer

# %% exceptions


@dataclass(kw_only=True)
class LayerWithoutCirkitCounterpartError(DataclassException, TypeError):
    """
    Exception raised when a layer of a layered circuit computes something no cirkit
    layer computes, so the circuit cannot be trained or queried with cirkit.
    """

    layer: Layer
    """
    The layer.
    """

    def error_message(self) -> str:
        return (
            f"The {type(self.layer).__name__} with {self.layer.number_of_nodes} units "
            f"has no cirkit counterpart."
        )

    def suggest_correction(self) -> str:
        return (
            "Convert only circuits built from a region graph: Gaussian and dense "
            "symbolic input layers, sums over every unit of child layers of equal "
            "size, and Hadamard or Kronecker products."
        )

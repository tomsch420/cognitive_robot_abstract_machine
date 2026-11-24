"""
Quantities that flow from the root of a layered circuit to its input layers: the expected
flows of a batch of data and the top-down probabilities of the nodes.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
from typing_extensions import Self

from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    InnerLayer,
    Layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)


# %% values of nodes and edges


@dataclass(kw_only=True)
class NodeAndEdgeValues:
    """
    One value for every node of every layer of a circuit and for every edge of every sum
    layer, on the host.
    """

    node_values: dict[Layer, np.ndarray] = field(default_factory=dict)
    """
    The value of every node of a layer, shape (#nodes,).
    """

    edge_values: dict[SumLayer, np.ndarray] = field(default_factory=dict)
    """
    The value of every edge of a sum layer, in the order of its edges, shape (#edges,).
    """


# %% top-down passes


@dataclass(kw_only=True)
class TopDownPass(ABC):
    """
    A quantity that starts at the root node and is passed on from every node to its
    children: a product node passes its value to every child unchanged, a sum node
    splits it into one value per edge, and a node receives the sum over its parents.
    """

    root: Layer
    """
    The root layer of the circuit, with a single node.
    """

    @property
    @abstractmethod
    def root_value(self) -> float:
        """
        :return: The value of the root node.
        """
        raise NotImplementedError

    @abstractmethod
    def sum_edge_values(
        self, *, layer: SumLayer, node_values: np.ndarray
    ) -> np.ndarray:
        """
        :param layer: A sum layer.
        :param node_values: The value of every node of the layer.
        :return: The value every edge of the layer passes on to its child node.
        """
        raise NotImplementedError

    def compute(self) -> NodeAndEdgeValues:
        """
        :return: The value of every node and every sum edge of the circuit.
        """
        layers = self.root.all_layers()
        result = NodeAndEdgeValues(
            node_values={layer: np.zeros(layer.number_of_nodes) for layer in layers}
        )
        result.node_values[self.root][:] = self.root_value
        for layer in layers:
            if not isinstance(layer, InnerLayer):
                continue
            node_values = result.node_values[layer]
            if isinstance(layer, SumLayer):
                edge_values = self.sum_edge_values(layer=layer, node_values=node_values)
                result.edge_values[layer] = edge_values
            else:
                edge_values = node_values[layer.edges.rows]
            received = np.zeros(int(layer.column_offsets[-1]))
            np.add.at(received, layer.edges.columns, edge_values)
            for child_layer, start, end in zip(
                layer.child_layers, layer.column_offsets[:-1], layer.column_offsets[1:]
            ):
                result.node_values[child_layer] += received[start:end]
        return result


@dataclass(kw_only=True)
class TopDownProbabilities(TopDownPass):
    """
    The probability that top-down sampling from the root reaches a node, the TD-prob
    of :cite:`liu2026rethinking`, definition 2.
    """

    @property
    def root_value(self) -> float:
        return 1.0

    def sum_edge_values(
        self, *, layer: SumLayer, node_values: np.ndarray
    ) -> np.ndarray:
        weights = HasParameterArrays.host_copy(
            layer.edges.edge_values(stored=layer.normalized_weights)
        )
        return weights * node_values[layer.edges.rows]


@dataclass(kw_only=True)
class ExpectedFlows(TopDownPass):
    """
    The expected flows of :cite:`choi2020probabilistic` summed over a batch of rows: how
    often a node or a sum edge is responsible for a row, in expectation over the latent
    variables of the circuit.
    """

    number_of_rows: int
    """
    The number of rows of the batch, the flow of the root.
    """

    log_weight_gradients: dict[SumLayer, np.ndarray]
    """
    The gradient of the summed log-likelihood of the batch with respect to the
    logarithmic weights of every sum layer, in the order of its edges.
    """

    @property
    def root_value(self) -> float:
        return float(self.number_of_rows)

    def sum_edge_values(
        self, *, layer: SumLayer, node_values: np.ndarray
    ) -> np.ndarray:
        # the forward pass normalizes the weights, so the gradient of a log-weight is the
        # flow of its edge minus its weight times the flow of its node; rounding can make
        # a vanishing flow slightly negative, which no flow is
        weights = HasParameterArrays.host_copy(
            layer.edges.edge_values(stored=layer.normalized_weights)
        )
        return np.maximum(
            self.log_weight_gradients[layer] + weights * node_values[layer.edges.rows],
            0.0,
        )

    @classmethod
    def from_parameter_gradients(
        cls, *, root: Layer, gradients: list[np.ndarray], number_of_rows: int
    ) -> Self:
        """
        :param root: The root layer of the circuit.
        :param gradients: The gradient of the summed log-likelihood of the batch with
            respect to every array of :attr:`Layer.circuit_parameters`.
        :param number_of_rows: The number of rows of the batch.
        :return: The expected flows of the batch.
        """
        return cls(
            root=root,
            number_of_rows=number_of_rows,
            log_weight_gradients={
                layer: HasParameterArrays.host_copy(
                    layer.edges.edge_values(stored=layer_gradients[0])
                )
                for layer, layer_gradients in CircuitParameterLayout(root=root)
                .split(arrays=gradients)
                .items()
                if isinstance(layer, SumLayer)
            },
        )


# %% parameter arrays per layer


@dataclass(kw_only=True)
class CircuitParameterLayout:
    """
    The layout of :attr:`Layer.circuit_parameters`: which arrays of the flat list belong
    to which layer.
    """

    root: Layer
    """
    The root layer of the circuit.
    """

    def split(self, *, arrays: list) -> dict[Layer, list]:
        """
        :param arrays: Arrays in the order and with the shapes of the circuit's
            parameters.
        :return: The arrays of every layer that has parameters.
        """
        result = {}
        position = 0
        for layer in self.root.all_layers():
            number_of_arrays = len(layer.parameters)
            if number_of_arrays:
                result[layer] = arrays[position : position + number_of_arrays]
            position += number_of_arrays
        return result

    def join(self, *, arrays_of_layers: dict[Layer, list]) -> list:
        """
        :param arrays_of_layers: The arrays of every layer that has parameters; a layer
            left out keeps its own parameters.
        :return: The arrays in the order of the circuit's parameters.
        """
        return [
            array
            for layer in self.root.all_layers()
            for array in arrays_of_layers.get(layer, layer.parameters)
        ]

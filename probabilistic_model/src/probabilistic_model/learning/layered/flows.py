"""
Quantities that flow from the root of a layered circuit to its input layers, such as the
top-down probabilities of the nodes.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

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

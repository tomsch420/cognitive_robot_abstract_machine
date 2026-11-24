"""
The closed form maximization step of expectation maximization for every kind of layer
whose parameters it learns.

Every kind of layer describes its nodes by statistics that are averages per node: the
weights of a sum node, the probabilities of a categorical node, the first two moments of
a Gaussian node. A batch contributes the flow-weighted sums of the same statistics, so
old and new statistics of a node can be mixed with any two weights.
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
    Layer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    SparseEntries,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.discrete_layer import (
    DiscreteLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.probability_table import (
    DenseProbabilityTable,
)


# %% maximization of one kind of layer


@dataclass(kw_only=True)
class LayerMaximization(ABC):
    """
    The maximization step of one kind of layer, on flat arrays of statistics in which
    every entry belongs to one node of the layer.
    """

    @abstractmethod
    def handles(self, *, layer: Layer) -> bool:
        """
        :param layer: A layer of the circuit.
        :return: Whether this maximization learns the parameters of the layer.
        """
        raise NotImplementedError

    @abstractmethod
    def owners(self, *, layer: Layer) -> np.ndarray:
        """
        :param layer: A layer this maximization handles.
        :return: The node every entry of the statistics belongs to.
        """
        raise NotImplementedError

    @abstractmethod
    def current_statistics(self, *, layer: Layer) -> np.ndarray:
        """
        :param layer: A layer this maximization handles.
        :return: The statistics of the current parameters of every node.
        """
        raise NotImplementedError

    @abstractmethod
    def batch_sums(
        self,
        *,
        layer: Layer,
        gradients: list[np.ndarray],
        node_flows: np.ndarray,
        edge_flows: np.ndarray | None,
    ) -> np.ndarray:
        """
        :param layer: A layer this maximization handles.
        :param gradients: The gradient of the summed log-likelihood of the batch with
            respect to the parameter arrays of the layer.
        :param node_flows: The expected flow of every node of the layer on the batch.
        :param edge_flows: The expected flow of every edge of a sum layer, else
            ``None``.
        :return: The flow-weighted sums of the statistics over the batch.
        """
        raise NotImplementedError

    def normalized_statistics(
        self,
        *,
        layer: Layer,
        sums: np.ndarray,
        node_flows: np.ndarray,
        number_of_rows: int,
    ) -> np.ndarray:
        """
        :param layer: A layer this maximization handles.
        :param sums: Flow-weighted sums of the statistics per row of the batch.
        :param node_flows: The flow of every node per row of the batch.
        :param number_of_rows: The number of rows of the batch.
        :return: The statistics the sums describe, the maximum likelihood estimate;
            ``nan`` for a node without flow.
        """
        owners = self.owners(layer=layer)
        with np.errstate(divide="ignore", invalid="ignore"):
            return sums / node_flows[owners]

    def child_entropies(
        self, *, layer: Layer, entropies: dict[Layer, np.ndarray]
    ) -> np.ndarray | None:
        """
        :param layer: A layer this maximization handles.
        :param entropies: The entropy of every node of every layer.
        :return: The entropy of the child of every entry, if the entries are weights of
            a mixture that entropy regularization can regularize, else ``None``.
        """
        return None

    @abstractmethod
    def maximizing_parameters(
        self, *, layer: Layer, statistics: np.ndarray
    ) -> list[np.ndarray]:
        """
        :param layer: A layer this maximization handles.
        :param statistics: The statistics of every node.
        :return: The parameter arrays of the layer that have these statistics.
        """
        raise NotImplementedError


@dataclass(kw_only=True)
class SumLayerMaximization(LayerMaximization):
    """
    The weights of a sum node are the normalized flows of its edges.
    """

    def handles(self, *, layer: Layer) -> bool:
        return isinstance(layer, SumLayer)

    def owners(self, *, layer: SumLayer) -> np.ndarray:
        return layer.edges.rows

    def current_statistics(self, *, layer: SumLayer) -> np.ndarray:
        return HasParameterArrays.host_copy(
            layer.edges.edge_values(stored=layer.normalized_weights)
        )

    def batch_sums(
        self,
        *,
        layer: SumLayer,
        gradients: list[np.ndarray],
        node_flows: np.ndarray,
        edge_flows: np.ndarray | None,
    ) -> np.ndarray:
        return edge_flows

    def child_entropies(
        self, *, layer: SumLayer, entropies: dict[Layer, np.ndarray]
    ) -> np.ndarray | None:
        child_entropies = np.concatenate(
            [entropies[child_layer] for child_layer in layer.child_layers]
        )
        return child_entropies[layer.edges.columns]

    def maximizing_parameters(
        self, *, layer: SumLayer, statistics: np.ndarray
    ) -> list[np.ndarray]:
        with np.errstate(divide="ignore"):
            log_weights = np.log(statistics)
        return [
            HasParameterArrays.host_copy(
                layer.edges.stored_values(
                    entries=SparseEntries(
                        log_weights, layer.edges.rows, layer.edges.columns
                    ),
                    padding=-np.inf,
                )
            )
        ]


@dataclass(kw_only=True)
class CategoricalLayerMaximization(LayerMaximization):
    """
    The probabilities of a categorical node are its normalized counts of the states,
    with a pseudocount added to every count.
    """

    pseudocount: float = 0.1
    """
    The count added to every state of every node before normalizing, which keeps a state
    the batch never shows possible.
    """

    def handles(self, *, layer: Layer) -> bool:
        return isinstance(layer, DiscreteLayer) and isinstance(
            layer.table, DenseProbabilityTable
        )

    def owners(self, *, layer: DiscreteLayer) -> np.ndarray:
        return np.repeat(np.arange(layer.number_of_nodes), layer.number_of_states)

    def current_statistics(self, *, layer: DiscreteLayer) -> np.ndarray:
        return np.exp(
            HasParameterArrays.host_copy(layer.table.log_probabilities)
        ).ravel()

    def batch_sums(
        self,
        *,
        layer: DiscreteLayer,
        gradients: list[np.ndarray],
        node_flows: np.ndarray,
        edge_flows: np.ndarray | None,
    ) -> np.ndarray:
        # the forward pass normalizes the log-probabilities per node, so the gradient
        # of a state is its expected count minus the flow of its node times its
        # probability
        (gradient,) = gradients
        probabilities = np.exp(
            HasParameterArrays.host_copy(layer.table.log_probabilities)
        )
        return (gradient + node_flows[:, np.newaxis] * probabilities).ravel()

    def normalized_statistics(
        self,
        *,
        layer: DiscreteLayer,
        sums: np.ndarray,
        node_flows: np.ndarray,
        number_of_rows: int,
    ) -> np.ndarray:
        smoothed = sums + self.pseudocount / number_of_rows
        totals = smoothed.reshape(layer.number_of_nodes, -1).sum(axis=1)
        with np.errstate(divide="ignore", invalid="ignore"):
            return smoothed / totals[self.owners(layer=layer)]

    def child_entropies(
        self, *, layer: DiscreteLayer, entropies: dict[Layer, np.ndarray]
    ) -> np.ndarray | None:
        return np.zeros(layer.number_of_nodes * layer.number_of_states)

    def maximizing_parameters(
        self, *, layer: DiscreteLayer, statistics: np.ndarray
    ) -> list[np.ndarray]:
        with np.errstate(divide="ignore"):
            return [np.log(statistics).reshape(layer.number_of_nodes, -1)]


@dataclass(kw_only=True)
class GaussianLayerMaximization(LayerMaximization):
    """
    The location and the scale of a Gaussian node follow from the flow-weighted first
    and second moment of its values.
    """

    def handles(self, *, layer: Layer) -> bool:
        return type(layer) is GaussianLayer

    def owners(self, *, layer: GaussianLayer) -> np.ndarray:
        return np.repeat(np.arange(layer.number_of_nodes), 2)

    def current_statistics(self, *, layer: GaussianLayer) -> np.ndarray:
        location = HasParameterArrays.host_copy(layer.location)
        scale = HasParameterArrays.host_copy(layer.scale)
        return np.stack([location, scale**2 + location**2], axis=1).ravel()

    def batch_sums(
        self,
        *,
        layer: GaussianLayer,
        gradients: list[np.ndarray],
        node_flows: np.ndarray,
        edge_flows: np.ndarray | None,
    ) -> np.ndarray:
        # the gradient of the log-density by location and log-scale is linear in the
        # flow-weighted sums of x and (x - location)^2, which are solved for here
        location_gradient, log_scale_gradient = gradients
        location = HasParameterArrays.host_copy(layer.location)
        scale = HasParameterArrays.host_copy(layer.scale)
        first = scale**2 * location_gradient + node_flows * location
        squared_deviation = scale**2 * (log_scale_gradient + node_flows)
        second = squared_deviation + 2 * location * first - location**2 * node_flows
        return np.stack([first, second], axis=1).ravel()

    def maximizing_parameters(
        self, *, layer: GaussianLayer, statistics: np.ndarray
    ) -> list[np.ndarray]:
        first, second = statistics.reshape(-1, 2).T
        variance = np.maximum(second - first**2, layer.minimum_scale**2)
        return [first, np.log(variance) / 2]


# %% all maximizations


@dataclass(kw_only=True)
class LayerMaximizations:
    """
    The maximization steps of every kind of layer expectation maximization learns.
    """

    maximizations: list[LayerMaximization] = field(
        default_factory=lambda: [
            SumLayerMaximization(),
            CategoricalLayerMaximization(),
            GaussianLayerMaximization(),
        ]
    )
    """
    The maximization steps, the first that handles a layer is used for it.
    """

    def find(self, *, layer: Layer) -> LayerMaximization | None:
        """
        :param layer: A layer of the circuit.
        :return: The maximization step of the layer, ``None`` if its parameters stay
            fixed.
        """
        return next(
            (
                maximization
                for maximization in self.maximizations
                if maximization.handles(layer=layer)
            ),
            None,
        )

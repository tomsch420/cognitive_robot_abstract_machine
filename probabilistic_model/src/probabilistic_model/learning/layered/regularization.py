"""
Data softening and entropy regularization of :cite:`liu2021tractable`, which regularize
tensorized expectation maximization of layered circuits.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass, field

import numpy as np
from array_api_compat import array_namespace

from probabilistic_model.learning.layered.exceptions import (
    UnsupportedEntropyLayerError,
)
from probabilistic_model.learning.layered.flows import TopDownPass
from probabilistic_model.learning.layered.statistics_adjustment import (
    LearnedLayerStatistics,
    StatisticsAdjustment,
)
from probabilistic_model.learning.parameters.expectation_maximization.layer_maximization import (
    EdgeCounts,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    BackendArray,
)
from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    InnerLayer,
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


# %% data softening


@dataclass(kw_only=True)
class DataSoftening:
    """
    Trains on the softened data of :cite:`liu2021tractable`, section 4.2: every observed
    discrete value keeps its state with probability :attr:`keep_probability` and moves
    to one of the other states uniformly otherwise.

    The softened likelihood of a categorical node is the likelihood of a node whose
    probabilities are mixed with those of the other states, so the expected counts of
    the softened data are the gradient through that mixture.
    """

    keep_probability: float
    """
    The probability that a value keeps its state, called beta in the paper.
    """

    def transformed(
        self, *, root: Layer, parameters: list[BackendArray]
    ) -> list[BackendArray]:
        """
        :param root: The root layer of the circuit.
        :param parameters: Arrays in the order of :attr:`Layer.circuit_parameters`.
        :return: The arrays whose likelihood is the likelihood of the softened data.
        """
        return [
            array
            for layer, arrays in zip(
                root.all_layers(), root.parameters_by_layer(parameters=parameters)
            )
            for array in self.softened_arrays(layer=layer, arrays=arrays)
        ]

    def softened_arrays(
        self, *, layer: Layer, arrays: list[BackendArray]
    ) -> list[BackendArray]:
        """
        :param layer: A layer of the circuit.
        :param arrays: Parameter arrays of the layer.
        :return: The arrays of a discrete layer with a dense table and several states
            softened, every other layer's arrays unchanged.
        """
        if not (
            isinstance(layer, DiscreteLayer)
            and isinstance(layer.table, DenseProbabilityTable)
            and layer.number_of_states > 1
        ):
            return arrays
        (log_probabilities,) = arrays
        namespace = array_namespace(log_probabilities)
        probabilities = namespace.exp(log_probabilities)
        # the mass of the other states is summed rather than taken as one minus the
        # state's own, so that the gradient reaches every state a value may come from
        totals = namespace.sum(probabilities, axis=1, keepdims=True)
        other = (totals - probabilities) / (layer.number_of_states - 1)
        return [
            namespace.log(
                self.keep_probability * probabilities
                + (1 - self.keep_probability) * other
            )
        ]


# %% entropies


@dataclass(kw_only=True)
class NodeEntropies:
    """
    The entropy of every node of a circuit over its observed variables and the latent
    variables of the sum nodes below it, which is exact for every smooth and
    decomposable circuit since the latent variables make it deterministic.
    """

    root: Layer
    """
    The root layer of the circuit.
    """

    def compute(self) -> dict[Layer, np.ndarray]:
        """
        :return: The entropy of every node of every layer.
        :raises UnsupportedEntropyLayerError: If an input layer has no closed form
            entropy.
        """
        result: dict[Layer, np.ndarray] = {}
        for layer in reversed(self.root.all_layers()):
            if not isinstance(layer, InnerLayer):
                result[layer] = self.input_layer_entropies(layer=layer)
                continue
            child_entropies = np.concatenate(
                [result[child_layer] for child_layer in layer.child_layers]
            )[layer.edges.columns]
            if isinstance(layer, SumLayer):
                weights = HasParameterArrays.host_copy(
                    layer.edges.edge_values(stored=layer.normalized_weights)
                )
                with np.errstate(divide="ignore", invalid="ignore"):
                    terms = np.where(
                        weights > 0, weights * (child_entropies - np.log(weights)), 0.0
                    )
            else:
                terms = child_entropies
            entropies = np.zeros(layer.number_of_nodes)
            np.add.at(entropies, layer.edges.rows, terms)
            result[layer] = entropies
        return result

    @staticmethod
    def input_layer_entropies(*, layer: Layer) -> np.ndarray:
        """
        :param layer: An input layer.
        :return: The entropy of every node of the layer.
        :raises UnsupportedEntropyLayerError: If the layer has no closed form entropy.
        """
        if isinstance(layer, DiscreteLayer):
            probabilities = layer.table.dense_probabilities()
            with np.errstate(divide="ignore", invalid="ignore"):
                terms = np.where(
                    probabilities > 0, -probabilities * np.log(probabilities), 0.0
                )
            return terms.sum(axis=1)
        if type(layer) is GaussianLayer:
            scale = HasParameterArrays.host_copy(layer.scale)
            return 0.5 * np.log(2 * np.pi * np.e * scale**2)
        raise UnsupportedEntropyLayerError(layer_type=type(layer))


# %% entropy regularization


@dataclass(kw_only=True)
class EntropyRegularization(StatisticsAdjustment):
    """
    The entropy regularization of :cite:`liu2021tractable`, algorithm 3: the weights of
    every mixture maximize the average log-likelihood plus :attr:`strength` times the
    entropy of the circuit, one mixture at a time, given the entropies of its children.
    """

    strength: float
    """
    The weight of the entropy in the objective, called tau in the paper.
    """

    bisection_steps: int = 60
    """
    The number of halvings of the interval of the Lagrange multiplier of a mixture.
    """

    newton_steps: int = 30
    """
    The number of Newton steps that solve for one weight given the multiplier.
    """

    maximum_newton_step: float = 5.0
    """
    The largest change of a log-weight in one Newton step, which keeps the exponential
    of the negated log-weight finite.
    """

    def adjusted(self, *, statistics: LearnedLayerStatistics) -> LearnedLayerStatistics:
        """
        Replace the counts of every sum layer by those whose maximization is the
        regularized weights, see :meth:`StatisticsAdjustment.adjusted`.
        """
        regularized = EntropyRegularizedTopDownProbabilities(
            root=statistics.root,
            regularization=self,
            edge_counts={
                layer: layer_statistics
                for layer, layer_statistics in zip(
                    statistics.layers, statistics.statistics
                )
                if isinstance(layer_statistics, EdgeCounts)
            },
            entropies=NodeEntropies(root=statistics.root).compute(),
            number_of_rows=statistics.number_of_rows,
        )
        regularized.compute()
        return statistics.with_statistics(
            statistics=[
                regularized.regularized_counts.get(layer, layer_statistics)
                for layer, layer_statistics in zip(
                    statistics.layers, statistics.statistics
                )
            ]
        )

    def regularized_weights(
        self,
        *,
        average_counts: np.ndarray,
        owners: np.ndarray,
        top_down_probabilities: np.ndarray,
        child_entropies: np.ndarray,
    ) -> np.ndarray:
        """
        Solve equation 5 of the paper for the weights of every mixture.

        :param average_counts: The expected count of every weight divided by the number
            of rows, called d in the paper.
        :param owners: The mixture every weight belongs to.
        :param top_down_probabilities: The top-down probability of every mixture.
        :param child_entropies: The entropy of the child every weight leads to.
        :return: The regularized weights, which sum to one per mixture.
        """
        number_of_mixtures = len(top_down_probabilities)
        pull = self.strength * top_down_probabilities[owners]
        children_per_mixture = np.bincount(owners, minlength=number_of_mixtures)
        log_children = np.log(np.maximum(children_per_mixture, 1))[owners]
        # every weight is at least one where the multiplier is at its lower bound and
        # at most one over the number of children at its upper bound
        lower = np.full(number_of_mixtures, np.inf)
        upper = np.full(number_of_mixtures, -np.inf)
        np.minimum.at(lower, owners, average_counts + pull * child_entropies)
        np.maximum.at(
            upper,
            owners,
            average_counts * np.exp(log_children)
            + pull * (log_children + child_entropies),
        )
        log_weights = np.zeros_like(average_counts)
        for _ in range(self.bisection_steps):
            multiplier = (lower + upper) / 2
            log_weights = self.log_weights_given_multiplier(
                average_counts=average_counts,
                pull=pull,
                child_entropies=child_entropies,
                multiplier=multiplier[owners],
                start=log_weights,
            )
            totals = np.bincount(
                owners, weights=np.exp(log_weights), minlength=number_of_mixtures
            )
            too_large = totals > 1
            lower = np.where(too_large, multiplier, lower)
            upper = np.where(too_large, upper, multiplier)
        weights = np.exp(log_weights)
        totals = np.bincount(owners, weights=weights, minlength=number_of_mixtures)
        count_totals = np.bincount(
            owners, weights=average_counts, minlength=number_of_mixtures
        )
        # without pull the objective is the likelihood alone, maximized by the counts
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(
                pull > 0,
                weights / totals[owners],
                average_counts / count_totals[owners],
            )

    def log_weights_given_multiplier(
        self,
        *,
        average_counts: np.ndarray,
        pull: np.ndarray,
        child_entropies: np.ndarray,
        multiplier: np.ndarray,
        start: np.ndarray,
    ) -> np.ndarray:
        """
        :return: The logarithm of every weight that solves
            ``d * exp(-u) - pull * u + pull * entropy = multiplier`` for ``u``.
        """
        log_weights = np.minimum(start, 0.0)
        for _ in range(self.newton_steps):
            exponential = average_counts * np.exp(-log_weights)
            residual = (
                exponential - pull * log_weights + pull * child_entropies - multiplier
            )
            slope = -exponential - pull
            with np.errstate(divide="ignore", invalid="ignore"):
                step = np.where(slope < 0, residual / slope, 0.0)
            log_weights = log_weights - np.clip(
                step, -self.maximum_newton_step, self.maximum_newton_step
            )
        return log_weights


@dataclass(kw_only=True)
class EntropyRegularizedTopDownProbabilities(TopDownPass):
    """
    The top-down probabilities of the circuit with the entropy-regularized weights,
    which every mixture is regularized with once the regularized weights above it are
    known, :cite:`liu2021tractable`, algorithm 3.
    """

    regularization: EntropyRegularization
    """
    The regularization whose weights are solved for.
    """

    edge_counts: dict[SumLayer, EdgeCounts]
    """
    The counts of every sum layer since the last update.
    """

    entropies: dict[Layer, np.ndarray]
    """
    The entropy of every node at the current parameters.
    """

    number_of_rows: int
    """
    The number of rows the counts were counted on.
    """

    regularized_counts: dict[SumLayer, EdgeCounts] = field(
        default_factory=dict, init=False
    )
    """
    The counts of every sum layer whose maximization is its regularized weights, filled
    by :meth:`compute`.
    """

    @property
    def root_value(self) -> float:
        return 1.0

    def sum_edge_values(
        self, *, layer: SumLayer, node_values: np.ndarray
    ) -> np.ndarray:
        rows, columns = layer.edges.rows, layer.edges.columns
        # rounding can make a vanishing count slightly negative, which no count is
        counts = np.maximum(
            layer.edges.edge_values(stored=self.edge_counts[layer].counts), 0.0
        )
        child_entropies = np.concatenate(
            [self.entropies[child_layer] for child_layer in layer.child_layers]
        )[columns]
        weights = np.nan_to_num(
            self.regularization.regularized_weights(
                average_counts=counts / self.number_of_rows,
                owners=rows,
                top_down_probabilities=node_values,
                child_entropies=child_entropies,
            )
        )
        node_counts = np.bincount(rows, weights=counts, minlength=layer.number_of_nodes)
        self.regularized_counts[layer] = EdgeCounts(
            counts=layer.edges.stored_values(
                entries=SparseEntries(weights * node_counts[rows], rows, columns),
                padding=0.0,
            )
        )
        return weights * node_values[rows]

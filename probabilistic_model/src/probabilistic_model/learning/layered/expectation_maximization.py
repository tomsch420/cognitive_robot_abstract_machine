"""
Expectation maximization of the parameters of layered circuits on the array backends
that differentiate: full batch, mini-batch :cite:`peharz2020einsum` and anemone
:cite:`liu2026rethinking`.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np
import tqdm

from probabilistic_model.learning.layered.flows import (
    CircuitParameterLayout,
    ExpectedFlows,
    NodeAndEdgeValues,
    TopDownProbabilities,
)
from probabilistic_model.learning.layered.gradients import LogLikelihoodGradient
from probabilistic_model.learning.layered.maximization import (
    LayerMaximization,
    LayerMaximizations,
)
from probabilistic_model.learning.layered.regularization import (
    DataSoftening,
    EntropyRegularization,
    NodeEntropies,
)
from probabilistic_model.probabilistic_circuit.tensorized.array_types import (
    SampleArray,
)
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
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)


# %% mixing of old and new statistics


@dataclass(kw_only=True)
class MixingWeights:
    """
    How much the current statistics and the statistics of a batch count in the update of
    every node of a layer.
    """

    current: np.ndarray
    """
    The weight of the current statistics of every node.
    """

    batch: np.ndarray
    """
    The weight of the statistics of the batch of every node.
    """


@dataclass(kw_only=True)
class Momentum:
    """
    The bias-corrected exponential moving average of the batch statistics of every
    layer, the momentum update of :cite:`liu2026rethinking`, section 5.1.
    """

    decay: float
    """
    The weight of the previous average, called beta in the paper.
    """

    averages: dict[int, list[np.ndarray]] = field(default_factory=dict)
    """
    The current averages of the statistics, by the position of their layer.
    """

    number_of_updates: int = 0
    """
    How many batches the averages have seen.
    """

    def averaged(self, *, position: int, values: list[np.ndarray]) -> list[np.ndarray]:
        """
        :param position: The position of the layer in the order of its circuit.
        :param values: The statistics of the current batch.
        :return: The bias-corrected averages including the current batch.
        """
        previous = self.averages.get(position, [np.zeros_like(value) for value in values])
        current = [
            self.decay * average + (1 - self.decay) * value
            for average, value in zip(previous, values)
        ]
        self.averages[position] = current
        correction = 1 - self.decay ** (self.number_of_updates + 1)
        return [average / correction for average in current]


# %% expectation maximization


@dataclass(kw_only=True)
class ExpectationMaximization(ABC):
    """
    Learns the parameters of a layered circuit by expectation maximization, reading the
    expected flows off the gradient of the log-likelihood.

    Sum layers, categorical layers with dense tables and Gaussian layers are learned;
    every other layer keeps its parameters.
    """

    epochs: int = 10
    """
    The number of passes over the data.
    """

    batch_size: int | None = None
    """
    The number of rows per update, all rows if ``None``.
    """

    momentum: float = 0.0
    """
    The decay of the moving average of the batch statistics, none if zero.
    """

    maximizations: LayerMaximizations = field(default_factory=LayerMaximizations)
    """
    The maximization step of every kind of layer.
    """

    data_softening: DataSoftening | None = None
    """
    The softening of the data, none if ``None``.
    """

    entropy_regularization: EntropyRegularization | None = None
    """
    The entropy regularization of the weights, none if ``None``.
    """

    seed: int = 0
    """
    The seed of the order in which the rows are visited.
    """

    average_log_likelihoods: list[float] = field(default_factory=list, init=False)
    """
    The average log-likelihood of the training rows in every epoch, before the updates
    of that epoch.
    """

    @abstractmethod
    def mixing_weights(
        self, *, node_rates: np.ndarray, top_down_probabilities: np.ndarray
    ) -> MixingWeights:
        """
        :param node_rates: The expected flow of every node of a layer per row of the
            batch.
        :param top_down_probabilities: The top-down probability of every node at the
            current parameters.
        :return: How the current and the batch statistics of every node are mixed.
        """
        raise NotImplementedError

    def fit(
        self, *, circuit: LayeredProbabilisticCircuit, data: SampleArray
    ) -> LayeredProbabilisticCircuit:
        """
        Learn the parameters of the circuit, in place.

        :param circuit: A circuit whose parameters are jax or torch arrays.
        :param data: The rows, whose symbolic values are hashes of domain elements.
        :return: The circuit.
        """
        encoded = np.asarray(HasParameterArrays.host_copy(circuit.encoded(data)))
        number_of_rows = len(encoded)
        batch_size = number_of_rows if self.batch_size is None else self.batch_size
        gradient = LogLikelihoodGradient.from_root(
            root=circuit.root, transformation=self.data_softening
        )
        momentum = Momentum(decay=self.momentum) if self.momentum > 0 else None
        random_number_generator = np.random.default_rng(self.seed)
        root = circuit.root
        for _ in tqdm.tqdm(range(self.epochs), desc="Expectation maximization"):
            order = random_number_generator.permutation(number_of_rows)
            summed_log_likelihood = 0.0
            for start in range(0, number_of_rows, batch_size):
                batch = encoded[order[start : start + batch_size]]
                gradient.root = root
                result = gradient.compute(
                    data=root.namespace.asarray(
                        batch, dtype=root.floating_point_type, device=root.device
                    )
                )
                summed_log_likelihood += result.log_likelihood
                flows = ExpectedFlows.from_parameter_gradients(
                    root=root, gradients=result.gradients, number_of_rows=len(batch)
                ).compute()
                root = self.updated_root(
                    root=root,
                    gradients=result.gradients,
                    flows=flows,
                    number_of_rows=len(batch),
                    momentum=momentum,
                )
                if momentum is not None:
                    momentum.number_of_updates += 1
            self.average_log_likelihoods.append(summed_log_likelihood / number_of_rows)
        circuit.root = root
        return circuit

    def updated_root(
        self,
        *,
        root: Layer,
        gradients: list[np.ndarray],
        flows: NodeAndEdgeValues,
        number_of_rows: int,
        momentum: Momentum | None,
    ) -> Layer:
        """
        :param root: The root layer at the current parameters.
        :param gradients: The gradient of the summed log-likelihood of the batch.
        :param flows: The expected flows of the batch.
        :param number_of_rows: The number of rows of the batch.
        :param momentum: The moving average of the batch statistics, if any.
        :return: The root layer of the circuit with the updated parameters.
        """
        layout = CircuitParameterLayout(root=root)
        gradients_of_layers = layout.split(arrays=gradients)
        top_down = TopDownProbabilities(root=root).compute().node_values
        entropies = (
            NodeEntropies(root=root).compute()
            if self.entropy_regularization is not None
            else {}
        )
        # the regularization passes the top-down probability on with the new weights
        regularized_top_down = {
            layer: np.zeros(layer.number_of_nodes) for layer in root.all_layers()
        }
        regularized_top_down[root][:] = 1.0
        new_arrays = {}
        for position, layer in enumerate(root.all_layers()):
            maximization = self.maximizations.find(layer=layer)
            weights = None
            if maximization is not None:
                statistics = self.updated_statistics(
                    layer=layer,
                    position=position,
                    maximization=maximization,
                    gradients=gradients_of_layers.get(layer, []),
                    flows=flows,
                    number_of_rows=number_of_rows,
                    momentum=momentum,
                    top_down_probabilities=top_down[layer],
                    regularized_top_down_probabilities=regularized_top_down[layer],
                    entropies=entropies,
                )
                new_arrays[layer] = [
                    layer.namespace.asarray(
                        array, dtype=layer.floating_point_type, device=layer.device
                    )
                    for array in maximization.maximizing_parameters(
                        layer=layer, statistics=statistics
                    )
                ]
                weights = statistics
            self.pass_top_down_probabilities(
                layer=layer, weights=weights, values=regularized_top_down
            )
        return root.copy_with_circuit_parameters(
            layout.join(arrays_of_layers=new_arrays)
        )

    def updated_statistics(
        self,
        *,
        layer: Layer,
        position: int,
        maximization: LayerMaximization,
        gradients: list[np.ndarray],
        flows: NodeAndEdgeValues,
        number_of_rows: int,
        momentum: Momentum | None,
        top_down_probabilities: np.ndarray,
        regularized_top_down_probabilities: np.ndarray,
        entropies: dict[Layer, np.ndarray],
    ) -> np.ndarray:
        """
        :return: The statistics of every node of the layer after the update.
        """
        node_rates = flows.node_values[layer] / number_of_rows
        sums = (
            maximization.batch_sums(
                layer=layer,
                gradients=gradients,
                node_flows=flows.node_values[layer],
                edge_flows=flows.edge_values.get(layer),
            )
            / number_of_rows
        )
        if momentum is not None:
            sums, node_rates = momentum.averaged(
                position=position, values=[sums, node_rates]
            )
        batch_statistics = maximization.normalized_statistics(
            layer=layer, sums=sums, node_flows=node_rates, number_of_rows=number_of_rows
        )
        child_entropies = (
            None
            if self.entropy_regularization is None
            else maximization.child_entropies(layer=layer, entropies=entropies)
        )
        if child_entropies is not None:
            batch_statistics = self.entropy_regularization.regularized_weights(
                average_counts=sums,
                owners=maximization.owners(layer=layer),
                top_down_probabilities=regularized_top_down_probabilities,
                child_entropies=child_entropies,
            )
        mixing = self.mixing_weights(
            node_rates=node_rates, top_down_probabilities=top_down_probabilities
        )
        owners = maximization.owners(layer=layer)
        usable = np.isfinite(batch_statistics) & (node_rates[owners] > 0)
        batch_weight = np.where(usable, mixing.batch[owners], 0.0)
        current_weight = mixing.current[owners]
        total = current_weight + batch_weight
        current = maximization.current_statistics(layer=layer)
        mixed = current_weight * current + batch_weight * np.where(
            usable, batch_statistics, 0.0
        )
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(total > 0, mixed / total, current)

    @staticmethod
    def pass_top_down_probabilities(
        *, layer: Layer, weights: np.ndarray | None, values: dict[Layer, np.ndarray]
    ) -> None:
        """
        Add what the nodes of a layer pass on to the top-down probabilities of their
        children.

        :param layer: The layer, whose own top-down probabilities are complete.
        :param weights: The updated weights of every edge if the layer is a sum layer.
        :param values: The top-down probabilities of every layer, updated in place.
        """
        if not isinstance(layer, InnerLayer):
            return
        passed = values[layer][layer.edges.rows]
        if isinstance(layer, SumLayer):
            passed = passed * weights
        received = np.zeros(int(layer.column_offsets[-1]))
        np.add.at(received, layer.edges.columns, passed)
        for child_layer, start, end in zip(
            layer.child_layers, layer.column_offsets[:-1], layer.column_offsets[1:]
        ):
            values[child_layer] += received[start:end]


@dataclass(kw_only=True)
class FullBatchExpectationMaximization(ExpectationMaximization):
    """
    Replaces the statistics of every node by those of the batch, which is the whole data
    unless :attr:`batch_size` says otherwise.
    """

    def mixing_weights(
        self, *, node_rates: np.ndarray, top_down_probabilities: np.ndarray
    ) -> MixingWeights:
        return MixingWeights(
            current=np.zeros_like(node_rates), batch=np.ones_like(node_rates)
        )


@dataclass(kw_only=True)
class MiniBatchExpectationMaximization(ExpectationMaximization):
    """
    Moves the statistics of every node a fixed step towards those of the batch,
    :cite:`peharz2020einsum`, equation 6 of :cite:`liu2026rethinking`.
    """

    batch_size: int | None = 512
    step_size: float = 0.1
    """
    The fraction of the way to the batch statistics, called alpha in the papers.
    """

    def mixing_weights(
        self, *, node_rates: np.ndarray, top_down_probabilities: np.ndarray
    ) -> MixingWeights:
        return MixingWeights(
            current=np.full_like(node_rates, 1 - self.step_size),
            batch=np.full_like(node_rates, self.step_size),
        )


@dataclass(kw_only=True)
class Anemone(ExpectationMaximization):
    """
    Mini-batch expectation maximization with a learning rate per node that grows with
    the node's flow on the batch relative to its top-down probability,
    :cite:`liu2026rethinking`, equation 7.
    """

    batch_size: int | None = 512
    momentum: float = 0.9
    learning_rate: float = 0.1
    """
    The learning rate, called eta in the paper.
    """

    def mixing_weights(
        self, *, node_rates: np.ndarray, top_down_probabilities: np.ndarray
    ) -> MixingWeights:
        return MixingWeights(
            current=top_down_probabilities, batch=self.learning_rate * node_rates
        )

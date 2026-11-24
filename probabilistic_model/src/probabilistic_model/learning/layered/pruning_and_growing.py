"""
Structure learning of sparse layered circuits by pruning and growing,
:cite:`dang2022sparse`: prune the sum edges of least expected flow, double every unit
with noisy copies of its weights, and relearn the parameters.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass, field

import numpy as np

from probabilistic_model.learning.layered.expectation_maximization import (
    ExpectationMaximization,
    MiniBatchExpectationMaximization,
)
from probabilistic_model.learning.layered.flows import ExpectedFlows
from probabilistic_model.learning.layered.gradients import LogLikelihoodGradient
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
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    SparseEdgeMatrix,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.inner_layer_edge import (
    InnerLayerEdges,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.product_layer import (
    ProductLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)


# %% flows of a dataset


@dataclass(kw_only=True)
class DatasetEdgeFlows:
    """
    The expected flow of every sum edge of a circuit summed over a whole dataset, taken
    in batches.
    """

    root: Layer
    """
    The root layer of the circuit, whose parameters are jax or torch arrays.
    """

    batch_size: int = 1024
    """
    The number of rows per gradient evaluation.
    """

    def compute(self, *, data: np.ndarray) -> dict[int, np.ndarray]:
        """
        :param data: The encoded rows.
        :return: The flow of every edge of every sum layer, by the position of the layer
            in :meth:`Layer.all_layers`.
        """
        positions = {
            layer: position for position, layer in enumerate(self.root.all_layers())
        }
        gradient = LogLikelihoodGradient.from_root(root=self.root)
        totals: dict[int, np.ndarray] = {}
        for start in range(0, len(data), self.batch_size):
            batch = data[start : start + self.batch_size]
            result = gradient.compute(
                data=self.root.namespace.asarray(
                    batch, dtype=self.root.floating_point_type, device=self.root.device
                )
            )
            flows = ExpectedFlows.from_parameter_gradients(
                root=self.root, gradients=result.gradients, number_of_rows=len(batch)
            ).compute()
            for layer, edge_flows in flows.edge_values.items():
                position = positions[layer]
                totals[position] = totals.get(position, 0.0) + edge_flows
        return totals


# %% pruning


@dataclass(kw_only=True)
class Pruning:
    """
    Removes the given fraction of all sum edges, those of least expected flow on the
    data, the E-FLOW heuristic of the paper; every sum unit keeps its edge of largest
    flow.
    """

    pruned_fraction: float
    """
    The fraction of the sum edges to remove.
    """

    def pruned_root(self, *, root: Layer, edge_flows: dict[int, np.ndarray]) -> Layer:
        """
        :param root: The root layer of the circuit, with host parameters.
        :param edge_flows: The expected flow of every edge of every sum layer, by the
            position of the layer in :meth:`Layer.all_layers`.
        :return: The root layer of the pruned circuit.
        """
        positions = {
            layer: position for position, layer in enumerate(root.all_layers())
        }
        all_flows = np.concatenate(list(edge_flows.values()))
        threshold = np.quantile(all_flows, self.pruned_fraction)
        rebuilt: dict[Layer, Layer] = {}
        for layer in reversed(root.all_layers()):
            if not isinstance(layer, InnerLayer):
                rebuilt[layer] = layer
                continue
            child_layers = [rebuilt[child_layer] for child_layer in layer.child_layers]
            if not isinstance(layer, SumLayer):
                rebuilt[layer] = layer.copy_with_child_layers(child_layers)
                continue
            flows = edge_flows[positions[layer]]
            kept = flows > threshold
            best_of_node = np.full(layer.number_of_nodes, -np.inf)
            np.maximum.at(best_of_node, layer.edges.rows, flows)
            kept |= flows == best_of_node[layer.edges.rows]
            edges = layer.inner_layer_edges
            rebuilt[layer] = SumLayer.from_edges(
                child_layers,
                InnerLayerEdges(
                    edges.nodes[kept],
                    edges.child_layer_indices[kept],
                    edges.child_nodes[kept],
                ),
                HasParameterArrays.host_copy(layer.edge_log_weights)[kept],
                layer.number_of_nodes,
                SparseEdgeMatrix,
            )
        return rebuilt[root]


# %% growing


@dataclass(kw_only=True)
class Growing:
    """
    Doubles every unit of a circuit below its root, algorithm 3 of the paper: a copy of
    a product unit multiplies the copies of its children, both copies of a sum unit mix
    the original and the copied children with noisy copies of the original weights, and
    input units are copied unchanged.
    """

    noise_variance: float
    """
    The variance of the factor around one that every copied weight is multiplied with.
    """

    seed: int = 0
    """
    The seed of the noise.
    """

    def grown_root(self, *, root: Layer) -> Layer:
        """
        :param root: The root layer of the circuit, with a single node.
        :return: The root layer of the grown circuit, which mixes both copies of the
            children of the original root.
        """
        random_number_generator = np.random.default_rng(self.seed)
        grown: dict[Layer, Layer] = {}
        for layer in reversed(root.all_layers()):
            copies = 1 if layer is root else 2
            if not isinstance(layer, InnerLayer):
                grown[layer] = type(layer).concatenate([layer, layer])
            elif isinstance(layer, SumLayer):
                grown[layer] = self.grown_sum_layer(
                    layer=layer,
                    child_layers=[grown[child] for child in layer.child_layers],
                    copies=copies,
                    random_number_generator=random_number_generator,
                )
            else:
                grown[layer] = self.grown_product_layer(
                    layer=layer,
                    child_layers=[grown[child] for child in layer.child_layers],
                )
        return grown[root]

    @staticmethod
    def grown_product_layer(
        *, layer: ProductLayer, child_layers: list[Layer]
    ) -> ProductLayer:
        """
        :param layer: The original product layer.
        :param child_layers: The grown child layers, each the original nodes followed by
            their copies.
        :return: The product layer of the original units followed by their copies.
        """
        edges = layer.inner_layer_edges
        original_sizes = np.array(
            [child.number_of_nodes for child in layer.child_layers]
        )
        copied = InnerLayerEdges(
            edges.nodes + layer.number_of_nodes,
            edges.child_layer_indices,
            edges.child_nodes + original_sizes[edges.child_layer_indices],
        )
        return ProductLayer.from_edges(
            child_layers,
            InnerLayerEdges.concatenate([edges, copied]),
            2 * layer.number_of_nodes,
            type(layer.edges),
        )

    def grown_sum_layer(
        self,
        *,
        layer: SumLayer,
        child_layers: list[Layer],
        copies: int,
        random_number_generator: np.random.Generator,
    ) -> SumLayer:
        """
        :param layer: The original sum layer.
        :param child_layers: The grown child layers, each the original nodes followed by
            their copies.
        :param copies: How many copies of every unit the grown layer holds.
        :param random_number_generator: The source of the noise.
        :return: The sum layer whose every copy of a unit mixes the original and the
            copied children with noisy halves of the original weights.
        """
        edges = layer.inner_layer_edges
        weights = HasParameterArrays.host_copy(layer.normalized_edge_log_weights)
        original_sizes = np.array(
            [child.number_of_nodes for child in layer.child_layers]
        )
        shift = original_sizes[edges.child_layer_indices]
        parts = [
            InnerLayerEdges(
                edges.nodes + copy * layer.number_of_nodes,
                edges.child_layer_indices,
                edges.child_nodes + child_copy * shift,
            )
            for copy in range(copies)
            for child_copy in range(2)
        ]
        noise = random_number_generator.normal(
            1.0, np.sqrt(self.noise_variance), size=len(parts) * len(weights)
        )
        noise = np.clip(noise, np.finfo(float).tiny, None)
        log_weights = np.tile(weights - np.log(2.0), len(parts)) + np.log(noise)
        return SumLayer.from_edges(
            child_layers,
            InnerLayerEdges.concatenate(parts),
            log_weights,
            copies * layer.number_of_nodes,
            SparseEdgeMatrix,
        )


# %% pruning and growing


@dataclass(kw_only=True)
class PruningAndGrowing:
    """
    Alternates pruning, growing and parameter learning, starting from the given circuit;
    with validation data, it stops at the first iteration that lowers the validation
    log-likelihood and keeps the circuit before it.
    """

    parameter_learning: ExpectationMaximization = field(
        default_factory=lambda: MiniBatchExpectationMaximization(epochs=50)
    )
    """
    The parameter learning after every growing step, mini-batch expectation maximization
    for 50 epochs in the paper.
    """

    pruned_fraction: float = 0.75
    """
    The fraction of the sum edges every iteration prunes, which growing restores.
    """

    noise_variance: float = 0.1
    """
    The variance of the noise of growing, the smallest value the paper searched.
    """

    iterations: int = 3
    """
    The largest number of prune, grow and learn iterations.
    """

    batch_size: int = 1024
    """
    The number of rows per gradient evaluation of the edge flows.
    """

    seed: int = 0
    """
    The seed of the noise of growing.
    """

    def fit(
        self,
        *,
        circuit: LayeredProbabilisticCircuit,
        data: SampleArray,
        validation_data: SampleArray | None = None,
    ) -> LayeredProbabilisticCircuit:
        """
        Learn the structure and the parameters of the circuit, in place.

        :param circuit: The initial circuit, whose parameters are jax or torch arrays.
        :param data: The training rows, whose symbolic values are hashes.
        :param validation_data: The rows that decide when to stop, if any.
        :return: The circuit.
        """
        encoded = np.asarray(HasParameterArrays.host_copy(circuit.encoded(data)))
        best_root = circuit.root
        best_validation = (
            None
            if validation_data is None
            else float(circuit.log_likelihood(events=validation_data).mean())
        )
        for iteration in range(self.iterations):
            edge_flows = DatasetEdgeFlows(
                root=circuit.root, batch_size=self.batch_size
            ).compute(data=encoded)
            # the structure is rebuilt on the host and moved back to the backend after
            pruned = Pruning(pruned_fraction=self.pruned_fraction).pruned_root(
                root=circuit.to_namespace(np).root, edge_flows=edge_flows
            )
            backend_root = circuit.root
            circuit.root = Growing(
                noise_variance=self.noise_variance, seed=self.seed + iteration
            ).grown_root(root=pruned)
            circuit.root = circuit.to_namespace(
                backend_root.namespace,
                device=backend_root.device,
                floating_point_type=backend_root.floating_point_type,
            ).root
            self.parameter_learning.fit(circuit=circuit, data=data)
            if validation_data is None:
                best_root = circuit.root
                continue
            validation = float(circuit.log_likelihood(events=validation_data).mean())
            if validation < best_validation:
                break
            best_root, best_validation = circuit.root, validation
        circuit.root = best_root
        return circuit

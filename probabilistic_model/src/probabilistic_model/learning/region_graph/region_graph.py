"""
Region graphs, the random partitions of variables that randomized and tensorized sum-
product networks (RAT-SPNs) are built from.
"""

from __future__ import annotations

from abc import ABC
from collections import deque
from dataclasses import dataclass, field

import jax.numpy as jnp
import jax.random
import rustworkx as rx
import numpy as np
from jax.experimental.sparse import BCOO
from random_events.variable import Continuous, Symbolic, Variable
from sortedcontainers import SortedSet
from typing_extensions import Iterable, Self

from probabilistic_model.probabilistic_circuit.jax.inner_layer import (
    DifferentiableLayer,
    DifferentiableProductLayer,
    DifferentiableDenseSumLayer,
)
from probabilistic_model.probabilistic_circuit.jax.discrete_layer import (
    DifferentiableDiscreteLayer,
)
from probabilistic_model.probabilistic_circuit.jax.gaussian_layer import (
    DifferentiableGaussianLayer,
)
from probabilistic_model.probabilistic_circuit.jax.probabilistic_circuit import (
    DifferentiableLayeredCircuit,
    ClassificationCircuit,
)
from probabilistic_model.graph import DirectedAcyclicGraph
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import Layer
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    DenseEdgeMatrix,
    SparseEntries,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.product_layer import (
    ProductLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.discrete_layer import (
    SymbolicLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.probability_table import (
    DenseProbabilityTable,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)
from probabilistic_model.learning.region_graph.exceptions import (
    ClassVariableMismatchError,
    MultipleRootsError,
    UnequalFactorSizesError,
    UnsupportedRegionVariableError,
)


# %% nodes


@dataclass(eq=False)
class RegionGraphNode(ABC):
    """
    A node of a region graph, compared by identity.
    """

    layer: DifferentiableLayer | Layer | None = field(
        default=None, init=False, repr=False
    )
    """
    The layer that models this node in the circuit of the region graph, once
    :meth:`RegionGraph.to_probabilistic_circuit` or
    :meth:`RegionGraph.to_layered_probabilistic_circuit` built it: an input or sum layer
    for a region, a product layer for a partition.
    """


@dataclass(eq=False)
class Region(RegionGraphNode):
    """
    A region in a region graph.

    A region is a set of variables. Refer to this paper
    https://ml-research.github.io/papers/peharz2019uai_ratspns.pdf
    .
    """

    variables: SortedSet
    """
    The variables in the region.
    """

    def random_partition(
        self, random_number_generator: np.random.Generator, number_of_parts: int = 2
    ) -> list[Self]:
        """
        :param random_number_generator: The generator that shuffles the variables.
        :param number_of_parts: The number of regions to split this region into.
        :return: Regions of nearly equal size whose variables partition the variables of
            this region at random.
        """
        indices = random_number_generator.permutation(len(self.variables))
        return [
            Region(SortedSet([self.variables[index] for index in split]))
            for split in np.array_split(indices, number_of_parts)
        ]

    def __repr__(self) -> str:
        """
        :return: The names of the variables of this region as a set.
        """
        return "{" + ", ".join([v.name for v in self.variables]) + "}"


@dataclass(eq=False)
class Partition(RegionGraphNode):
    """
    A partition in a region graph.
    """


# %% region graph


@dataclass(eq=False)
class RegionGraph(DirectedAcyclicGraph[RegionGraphNode]):
    """
    A region graph is a directed acyclic bipartite graph that represents a (potentially
    repeated) partition of variables.

    Bluntly speaking, a region graph is an ensemble of random Latent-Variable trees.
    """

    variables: SortedSet
    """
    The variables of the region graph.
    """

    partitions: int = 2
    """
    The number of partitions to create in each region.
    """

    depth: int = 2
    """
    The depth of each repetition of the region graph.
    """

    repetitions: int = 2
    """
    The number of repetitions to create in the region graph.

    A repetition is another partitioning on the set of variables.
    """

    classes: int = 1
    """
    The number of classes.

    If classes > 1, the circuit is a classifier, if 1 it is a generative model.
    """

    seed: int = 69
    """
    The seed of :attr:`random_number_generator`, which fixes the graph.
    """

    random_number_generator: np.random.Generator = field(init=False, repr=False)
    """
    The generator that draws the random partitions.
    """

    def __post_init__(self):
        """
        Seed :attr:`random_number_generator` with :attr:`seed`.
        """
        self.random_number_generator = np.random.default_rng(self.seed)

    def create_random_region_graph(self) -> Self:
        """
        Create a random region graph according to the classes parameters in-place.

        :return: This region graph.
        """
        root = Region(self.variables)
        self.add_node(root)
        for repetition in range(self.repetitions):
            self.recursive_split(root)
        return self

    def regions(self) -> Iterable[Region]:
        """
        :return: An iterator over all regions.
        """
        for node in self.nodes():
            if isinstance(node, Region):
                yield node

    def partition_nodes(self) -> Iterable[Partition]:
        """
        :return: An iterator over all partitions.
        """
        for node in self.nodes():
            if isinstance(node, Partition):
                yield node

    def recursive_split(self, node: Region) -> None:
        """
        Recursively split a region into partitions.

        :param node: The node to start inducing from.
        """
        root_partition = Partition()
        self.add_edge(node, root_partition)
        remaining_regions = deque([(node, self.depth, root_partition)])

        while remaining_regions:
            region, depth, partition = remaining_regions.popleft()

            if depth == 0 or len(region.variables) == 1:
                for variable in region.variables:
                    self.add_edge(partition, Region(SortedSet([variable])))
                continue

            new_regions = region.random_partition(
                self.random_number_generator, self.partitions
            )
            for new_region in new_regions:
                self.add_edge(partition, new_region)
                new_partition = Partition()
                self.add_edge(new_region, new_partition)
                remaining_regions.append((new_region, depth - 1, new_partition))

    @property
    def root(self) -> Region:
        """
        :return: The root of the region graph.
        """
        possible_roots = [node for node in self.nodes() if self.in_degree(node) == 0]
        if len(possible_roots) > 1:
            raise MultipleRootsError(possible_roots)
        return possible_roots[0]

    def to_probabilistic_circuit(
        self,
        input_units: int = 5,
        sum_units: int = 5,
        key: jax.Array = jax.random.PRNGKey(69),
    ) -> DifferentiableLayeredCircuit | ClassificationCircuit:
        """
        Convert the region graph to a jax probabilistic circuit.

        :param input_units: The number of input units to use in each input layer.
        :param sum_units: The number of sum units to use in each sum layer.
        :param key: The random key to use for all trainable parameters.
        :return: The layered circuit in jax.
        """
        root = self.root

        # create nodes for each region
        for layer in reversed(rx.bfs_layers(self.graph, [self.index_of(root)])):
            for node in map(self.graph.__getitem__, layer):
                children = list(self.successors(node))
                parents = list(self.predecessors(node))
                if isinstance(node, Region):
                    # if the region is a leaf
                    if len(children) == 0:
                        variable = node.variables[0]
                        variable_index = self.variables.index(variable)
                        if isinstance(variable, Continuous):
                            location = jax.random.uniform(
                                key, shape=(input_units,), minval=-1.0, maxval=1.0
                            )
                            log_scale = jnp.log(
                                jax.random.uniform(
                                    key, shape=(input_units,), minval=0.5, maxval=3.0
                                )
                            )
                            node.layer = DifferentiableGaussianLayer(
                                variable_index,
                                location=location,
                                log_scale=log_scale,
                                minimum_scale=jnp.full_like(location, 0.1),
                            )
                            node.layer.validate()
                        elif isinstance(variable, Symbolic):
                            log_probabilities = jax.random.uniform(
                                key,
                                shape=(input_units, len(variable.domain.simple_sets)),
                                minval=0.1,
                                maxval=1.0,
                            )
                            log_probabilities = jnp.log(log_probabilities)
                            node.layer = DifferentiableDiscreteLayer(
                                variable_index, log_probabilities=log_probabilities
                            )
                        else:
                            raise UnsupportedRegionVariableError(variable)

                    # if the region is root or in the middle
                    else:
                        # if the region is root
                        if len(parents) == 0:
                            sum_units = self.classes

                        log_weights = [
                            jnp.log(
                                jax.random.uniform(
                                    key,
                                    shape=(sum_units, child.layer.number_of_nodes),
                                    minval=0.1,
                                    maxval=1.0,
                                )
                            )
                            for child in children
                        ]
                        node.layer = DifferentiableDenseSumLayer(
                            [child.layer for child in children], log_weights=log_weights
                        )
                        node.layer.validate()

                elif isinstance(node, Partition):
                    node_lengths = [child.layer.number_of_nodes for child in children]
                    if not len(set(node_lengths)) == 1:
                        raise UnequalFactorSizesError(node_lengths)

                    edges = (
                        jnp.arange(node_lengths[0])
                        .reshape(1, -1)
                        .repeat(len(children), axis=0)
                    )
                    sparse_edges = BCOO.fromdense(jnp.ones_like(edges))
                    sparse_edges.data = edges.flatten()
                    node.layer = DifferentiableProductLayer(
                        [child.layer for child in children], sparse_edges
                    )
                    node.layer.validate()

        if self.classes > 1:
            model = ClassificationCircuit(self.variables, root.layer)
        else:
            model = DifferentiableLayeredCircuit(self.variables, root.layer)

        return model

    # %% tensorized circuits

    def to_layered_probabilistic_circuit(
        self,
        input_units: int = 5,
        sum_units: int = 5,
        key: jax.Array = jax.random.PRNGKey(69),
        class_variable: Symbolic | None = None,
    ) -> LayeredProbabilisticCircuit:
        """
        Convert the region graph to a layered circuit with jax parameters, in the
        floating point type jax computes in by default.

        Every sum layer is fully connected and stores its edges densely. With more than
        one class, the root mixes one class conditional circuit per class, each
        multiplied by the indicator of its class, so that training the joint likelihood
        trains a classifier.

        :param input_units: The number of input units of each leaf region.
        :param sum_units: The number of sum units of each inner region.
        :param key: The random key of every initial parameter.
        :param class_variable: The variable whose domain elements are the classes, if
            :attr:`classes` is more than one.
        :return: The layered circuit.
        :raises ClassVariableMismatchError: If the class variable does not fit
            :attr:`classes`.
        """
        if self.classes > 1 and (
            class_variable is None
            or len(class_variable.domain.simple_sets) != self.classes
            or class_variable in self.variables
        ):
            raise ClassVariableMismatchError(self.classes, class_variable)
        variables = SortedSet(self.variables)
        if self.classes > 1:
            variables = variables | SortedSet([class_variable])
        keys = iter(jax.random.split(key, len(self.nodes()) + 1))

        for layer in reversed(rx.bfs_layers(self.graph, [self.index_of(self.root)])):
            for node in map(self.graph.__getitem__, layer):
                children = list(self.successors(node))
                if isinstance(node, Partition):
                    node.layer = self.product_of_children(children)
                elif children:
                    number_of_nodes = (
                        self.classes if self.in_degree(node) == 0 else sum_units
                    )
                    node.layer = self.fully_connected_sum_layer(
                        children, number_of_nodes, next(keys)
                    )
                else:
                    node.layer = self.input_layer(
                        node.variables[0], variables, input_units, next(keys)
                    )

        root = self.root.layer
        if self.classes > 1:
            root = self.mixture_of_classes(root, class_variable, variables)
        circuit = LayeredProbabilisticCircuit(variables, root)
        return circuit.to_namespace(jnp, dtype=jnp.result_type(float))

    @staticmethod
    def product_of_children(children: list[RegionGraphNode]) -> ProductLayer:
        """
        :param children: The regions a partition multiplies, all with as many units.
        :return: The product layer whose node ``i`` multiplies node ``i`` of every
            child.
        :raises UnequalFactorSizesError: If the children have different numbers of
            units.
        """
        node_lengths = [child.layer.number_of_nodes for child in children]
        if len(set(node_lengths)) != 1:
            raise UnequalFactorSizesError(node_lengths)
        return ProductLayer.node_wise_product_of([child.layer for child in children])

    @staticmethod
    def fully_connected_sum_layer(
        children: list[RegionGraphNode], number_of_nodes: int, key: jax.Array
    ) -> SumLayer:
        """
        :param children: The partitions a region mixes.
        :param number_of_nodes: The number of sum units of the region.
        :param key: The random key of the initial weights.
        :return: The sum layer that connects every unit to every unit of every child,
            with dense edges and weights drawn uniformly from [0.1, 1].
        """
        child_layers = [child.layer for child in children]
        number_of_columns = sum(layer.number_of_nodes for layer in child_layers)
        rows, columns = np.nonzero(np.ones((number_of_nodes, number_of_columns)))
        weights = np.asarray(
            jax.random.uniform(key, shape=(len(rows),), minval=0.1, maxval=1.0)
        )
        return SumLayer.from_entries(
            child_layers,
            SparseEntries(np.log(weights.astype(float)), rows, columns),
            number_of_nodes,
            DenseEdgeMatrix,
        )

    @staticmethod
    def input_layer(
        variable: Variable,
        variables: SortedSet,
        input_units: int,
        key: jax.Array,
    ) -> GaussianLayer | SymbolicLayer:
        """
        :param variable: The variable of a leaf region.
        :param variables: The variables of the circuit.
        :param input_units: The number of input units.
        :param key: The random key of the initial parameters.
        :return: Gaussians with locations in [-1, 1] and scales in [0.5, 3] for a
            continuous variable, categorical distributions with probabilities drawn
            from [0.1, 1] and normalized for a symbolic one.
        :raises UnsupportedRegionVariableError: If the variable is of another type.
        """
        variable_index = variables.index(variable)
        location_key, scale_key = jax.random.split(key)
        if isinstance(variable, Continuous):
            return GaussianLayer(
                variable_index,
                np.asarray(
                    jax.random.uniform(
                        location_key, (input_units,), minval=-1.0, maxval=1.0
                    )
                ).astype(float),
                np.asarray(
                    jax.random.uniform(
                        scale_key, (input_units,), minval=0.5, maxval=3.0
                    )
                ).astype(float),
            )
        if isinstance(variable, Symbolic):
            number_of_states = len(variable.domain.simple_sets)
            probabilities = np.asarray(
                jax.random.uniform(
                    location_key,
                    (input_units, number_of_states),
                    minval=0.1,
                    maxval=1.0,
                )
            ).astype(float)
            probabilities /= probabilities.sum(axis=1, keepdims=True)
            return SymbolicLayer(
                variable_index,
                np.arange(number_of_states),
                DenseProbabilityTable(np.log(probabilities)),
                SymbolicEncoding(variable).hashes,
            )
        raise UnsupportedRegionVariableError(variable)

    def mixture_of_classes(
        self, class_conditionals: Layer, class_variable: Symbolic, variables: SortedSet
    ) -> SumLayer:
        """
        :param class_conditionals: The layer with one node per class.
        :param class_variable: The variable whose domain elements are the classes.
        :param variables: The variables of the circuit.
        :return: The root that mixes, with equal weights, the product of every class
            conditional node with the indicator of its class.
        """
        with np.errstate(divide="ignore"):
            indicators = SymbolicLayer(
                variables.index(class_variable),
                np.arange(self.classes),
                DenseProbabilityTable(np.log(np.eye(self.classes))),
                SymbolicEncoding(class_variable).hashes,
            )
        products = ProductLayer.node_wise_product_of([class_conditionals, indicators])
        return SumLayer.from_entries(
            [products],
            SparseEntries(
                np.full(self.classes, -np.log(self.classes)),
                np.zeros(self.classes, dtype=np.int64),
                np.arange(self.classes),
            ),
            1,
            DenseEdgeMatrix,
        )

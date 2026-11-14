"""
Hidden Chow-Liu tree circuits of :cite:`liu2021tractable`: a Chow-Liu tree
:cite:`chow1968approximating` over the variables whose every variable is replaced by a
latent variable with a fixed number of states that the observed variable hangs below.
"""

from __future__ import annotations

# %% imports
from dataclasses import dataclass

import jax.numpy as jnp
import numpy as np
from random_events.variable import Continuous, Symbolic, Variable
from scipy.sparse.csgraph import breadth_first_order, minimum_spanning_tree
from sortedcontainers import SortedSet

from probabilistic_model.learning.layered.exceptions import (
    UnsupportedStructureVariableError,
)
from probabilistic_model.learning.layered.mutual_information import (
    DiscretizedData,
    PairwiseMutualInformation,
)
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    Layer,
)
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


# %% Chow-Liu tree


@dataclass(kw_only=True)
class ChowLiuTree:
    """
    The spanning tree over the variables that maximizes the summed pairwise mutual
    information, rooted at the first variable.
    """

    parents: np.ndarray
    """
    The parent of every variable, ``-1`` for the root.
    """

    order: np.ndarray
    """
    The variables in breadth-first order from the root, parents before children.
    """

    @classmethod
    def from_mutual_information(cls, *, mutual_information: np.ndarray) -> ChowLiuTree:
        """
        :param mutual_information: The symmetric matrix of pairwise mutual information.
        :return: The maximum spanning tree of the complete graph weighted by it.
        """
        # shift the weights to be positive, since the spanning tree reads zero as no edge
        costs = mutual_information.max() + 1.0 - mutual_information
        np.fill_diagonal(costs, 0.0)
        tree = minimum_spanning_tree(costs)
        order, predecessors = breadth_first_order(
            tree, i_start=0, directed=False, return_predecessors=True
        )
        parents = np.where(predecessors < 0, -1, predecessors)
        return cls(parents=parents, order=order)


# %% hidden Chow-Liu tree


@dataclass(kw_only=True)
class HiddenChowLiuTree:
    """
    Builds a hidden Chow-Liu tree circuit from data, with random parameters.

    The latent variable of every tree node becomes a layer of :attr:`hidden_states`
    product units; each multiplies an input distribution of its variable with one
    mixture over the units of every child, so a unit and its mixtures stand for one
    state of the latent variable and its transitions to the children's states.
    """

    variables: SortedSet
    """
    The variables of the circuit, one per column of the data.
    """

    hidden_states: int = 12
    """
    The number of states of every latent variable, 12 in the paper's twenty-dataset
    experiments.
    """

    number_of_bins: int = 10
    """
    The number of quantile bins a continuous variable is cut into for the mutual
    information.
    """

    seed: int = 0
    """
    The seed of the random initial parameters.
    """

    def build_circuit(self, *, data: np.ndarray) -> LayeredProbabilisticCircuit:
        """
        :param data: The rows, encoded as the circuits read them.
        :return: The hidden Chow-Liu tree circuit with jax parameters.
        """
        tree = ChowLiuTree.from_mutual_information(
            mutual_information=PairwiseMutualInformation(
                discretized=DiscretizedData(
                    variables=self.variables,
                    data=data,
                    number_of_bins=self.number_of_bins,
                )
            ).compute()
        )
        random_number_generator = np.random.default_rng(self.seed)
        units: dict[int, Layer] = {}
        for variable_index in tree.order[::-1]:
            factors = [
                self.input_layer(
                    variable_index=int(variable_index),
                    data=data,
                    random_number_generator=random_number_generator,
                )
            ]
            factors.extend(
                self.mixture_layer(
                    child_layer=units.pop(int(child)),
                    number_of_nodes=self.hidden_states,
                    random_number_generator=random_number_generator,
                )
                for child in np.flatnonzero(tree.parents == variable_index)
            )
            units[int(variable_index)] = (
                factors[0]
                if len(factors) == 1
                else ProductLayer.node_wise_product_of(factors)
            )
        root = self.mixture_layer(
            child_layer=units[int(tree.order[0])],
            number_of_nodes=1,
            random_number_generator=random_number_generator,
        )
        return LayeredProbabilisticCircuit(SortedSet(self.variables), root).to_namespace(
            jnp, floating_point_type=jnp.result_type(float)
        )

    @staticmethod
    def mixture_layer(
        *,
        child_layer: Layer,
        number_of_nodes: int,
        random_number_generator: np.random.Generator,
    ) -> SumLayer:
        """
        :param child_layer: The layer whose units are mixed.
        :param number_of_nodes: The number of mixtures.
        :param random_number_generator: The source of the initial weights.
        :return: A layer of mixtures over every unit of the child layer, with weights
            drawn uniformly from [0.1, 1].
        """
        rows, columns = np.nonzero(
            np.ones((number_of_nodes, child_layer.number_of_nodes))
        )
        weights = random_number_generator.uniform(0.1, 1.0, size=len(rows))
        return SumLayer.from_entries(
            [child_layer],
            SparseEntries(np.log(weights), rows, columns),
            number_of_nodes,
            DenseEdgeMatrix,
        )

    def input_layer(
        self,
        *,
        variable_index: int,
        data: np.ndarray,
        random_number_generator: np.random.Generator,
    ) -> Layer:
        """
        :param variable_index: The index of the variable.
        :param data: The rows, encoded as the circuits read them.
        :param random_number_generator: The source of the initial parameters.
        :return: One input distribution of the variable per state of its latent
            variable: categorical distributions with probabilities drawn from [0.1, 1],
            or Gaussians centered at random rows with the scale of the column.
        :raises UnsupportedStructureVariableError: If the variable is neither symbolic
            nor continuous.
        """
        variable: Variable = self.variables[variable_index]
        if isinstance(variable, Symbolic):
            number_of_states = len(variable.domain.simple_sets)
            probabilities = random_number_generator.uniform(
                0.1, 1.0, size=(self.hidden_states, number_of_states)
            )
            probabilities /= probabilities.sum(axis=1, keepdims=True)
            return SymbolicLayer(
                variable_index,
                np.arange(number_of_states),
                DenseProbabilityTable(np.log(probabilities)),
                SymbolicEncoding(variable).hashes,
            )
        if isinstance(variable, Continuous):
            column = data[:, variable_index]
            return GaussianLayer(
                variable_index,
                random_number_generator.choice(column, size=self.hidden_states),
                np.full(
                    self.hidden_states,
                    max(float(column.std()), GaussianLayer.minimum_scale),
                ),
            )
        raise UnsupportedStructureVariableError(variable=variable)

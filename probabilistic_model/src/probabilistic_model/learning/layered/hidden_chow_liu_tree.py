"""
Hidden Chow-Liu tree circuits of :cite:`liu2021tractable`: a Chow-Liu tree
:cite:`chow1968approximating` over the variables whose every variable is replaced by a
latent variable with a fixed number of states that the observed variable hangs below.
"""

from __future__ import annotations

# %% imports
from abc import ABC, abstractmethod
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


# %% parameters


@dataclass(kw_only=True)
class HiddenChowLiuTreeParameters(ABC):
    """
    Where the parameters of a hidden Chow-Liu tree circuit come from.
    """

    variables: SortedSet
    """
    The variables of the circuit, one per column of the data.
    """

    data: np.ndarray
    """
    The rows, encoded as the circuits read them.
    """

    hidden_states: int
    """
    The number of states of every latent variable.
    """

    @abstractmethod
    def symbolic_probabilities(self, *, variable_index: int) -> np.ndarray:
        """
        :param variable_index: The index of a symbolic variable.
        :return: The probability of every state of the variable given every state of its
            latent variable, shape (#hidden states, #states).
        """
        raise NotImplementedError

    @abstractmethod
    def gaussian_parameters(
        self, *, variable_index: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        :param variable_index: The index of a continuous variable.
        :return: The location and the scale of the Gaussian of the variable given every
            state of its latent variable.
        """
        raise NotImplementedError

    @abstractmethod
    def transition_weights(self, *, parent: int | None, child: int) -> np.ndarray:
        """
        :param parent: The index of the parent variable, ``None`` for the root.
        :param child: The index of the child variable.
        :return: The weight of every state of the child's latent variable given every
            state of the parent's, shape (#hidden states or 1 for the root, #hidden
            states).
        """
        raise NotImplementedError

    def input_layer(self, *, variable_index: int) -> Layer:
        """
        :param variable_index: The index of the variable.
        :return: One input distribution of the variable per state of its latent
            variable.
        :raises UnsupportedStructureVariableError: If the variable is neither symbolic
            nor continuous.
        """
        variable: Variable = self.variables[variable_index]
        if isinstance(variable, Symbolic):
            probabilities = self.symbolic_probabilities(variable_index=variable_index)
            with np.errstate(divide="ignore"):
                log_probabilities = np.log(probabilities)
            return SymbolicLayer(
                node_variables=np.full(len(probabilities), variable_index),
                states=np.arange(probabilities.shape[1]),
                table=DenseProbabilityTable(log_probabilities=log_probabilities),
            )
        if isinstance(variable, Continuous):
            location, scale = self.gaussian_parameters(variable_index=variable_index)
            return GaussianLayer(
                node_variables=np.full(len(location), variable_index),
                location=location,
                log_scale=np.log(scale),
            )
        raise UnsupportedStructureVariableError(variable=variable)


@dataclass(kw_only=True)
class RandomParameters(HiddenChowLiuTreeParameters):
    """
    Random initial parameters: categorical probabilities and transition weights drawn
    from [0.1, 1] and normalized, Gaussians centered at random rows with the scale of
    the column.
    """

    random_number_generator: np.random.Generator
    """
    The source of the parameters.
    """

    def symbolic_probabilities(self, *, variable_index: int) -> np.ndarray:
        number_of_states = len(self.variables[variable_index].domain.simple_sets)
        probabilities = self.random_number_generator.uniform(
            0.1, 1.0, size=(self.hidden_states, number_of_states)
        )
        return probabilities / probabilities.sum(axis=1, keepdims=True)

    def gaussian_parameters(
        self, *, variable_index: int
    ) -> tuple[np.ndarray, np.ndarray]:
        column = self.data[:, variable_index]
        return (
            self.random_number_generator.choice(column, size=self.hidden_states),
            np.full(
                self.hidden_states,
                max(float(column.std()), GaussianLayer.minimum_scale),
            ),
        )

    def transition_weights(self, *, parent: int | None, child: int) -> np.ndarray:
        rows = 1 if parent is None else self.hidden_states
        return self.random_number_generator.uniform(
            0.1, 1.0, size=(rows, self.hidden_states)
        )


@dataclass(kw_only=True)
class CountedParameters(HiddenChowLiuTreeParameters):
    """
    The maximum likelihood parameters of the rows completed with an assignment of every
    latent variable, step 3 of latent variable distillation :cite:`liu2023scaling`.
    """

    latent_states: np.ndarray
    """
    The state of every latent variable in every row, shape (#rows, #variables).
    """

    pseudocount: float = 0.1
    """
    The count added to every state and every transition.
    """

    def symbolic_probabilities(self, *, variable_index: int) -> np.ndarray:
        variable = self.variables[variable_index]
        states = (
            SymbolicEncoding(variable)
            .indices_of_hashes(self.data[:, variable_index])
            .astype(int)
        )
        counts = np.full(
            (self.hidden_states, len(variable.domain.simple_sets)), self.pseudocount
        )
        np.add.at(counts, (self.latent_states[:, variable_index], states), 1.0)
        return counts / counts.sum(axis=1, keepdims=True)

    def gaussian_parameters(
        self, *, variable_index: int
    ) -> tuple[np.ndarray, np.ndarray]:
        column = self.data[:, variable_index]
        latent = self.latent_states[:, variable_index]
        counts = np.bincount(latent, minlength=self.hidden_states)
        sums = np.bincount(latent, weights=column, minlength=self.hidden_states)
        squares = np.bincount(latent, weights=column**2, minlength=self.hidden_states)
        with np.errstate(divide="ignore", invalid="ignore"):
            location = np.where(counts > 0, sums / counts, column.mean())
            variance = np.where(
                counts > 1, squares / counts - location**2, column.var()
            )
        return location, np.sqrt(np.maximum(variance, GaussianLayer.minimum_scale**2))

    def transition_weights(self, *, parent: int | None, child: int) -> np.ndarray:
        if parent is None:
            counts = np.full((1, self.hidden_states), self.pseudocount)
            np.add.at(counts, (0, self.latent_states[:, child]), 1.0)
            return counts
        counts = np.full((self.hidden_states, self.hidden_states), self.pseudocount)
        np.add.at(
            counts,
            (self.latent_states[:, parent], self.latent_states[:, child]),
            1.0,
        )
        return counts


# %% hidden Chow-Liu tree


@dataclass(kw_only=True)
class HiddenChowLiuTree:
    """
    Builds a hidden Chow-Liu tree circuit from data.

    Every latent variable is a layer of :attr:`hidden_states` product units, each an input distribution of its variable
    times one mixture over the units of every child: one latent state and its
    transitions to the children's states.
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

    def build_circuit(
        self, *, data: np.ndarray, latent_states: np.ndarray | None = None
    ) -> LayeredProbabilisticCircuit:
        """
        :param data: The rows, encoded as the circuits read them.
        :param latent_states: The state of every latent variable in every row, if the
            parameters are to be counted from them; random parameters if ``None``.
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
        parameters = (
            RandomParameters(
                variables=self.variables,
                data=data,
                hidden_states=self.hidden_states,
                random_number_generator=np.random.default_rng(self.seed),
            )
            if latent_states is None
            else CountedParameters(
                variables=self.variables,
                data=data,
                hidden_states=self.hidden_states,
                latent_states=latent_states,
            )
        )
        units: dict[int, Layer] = {}
        for variable_index in tree.order[::-1]:
            factors = [parameters.input_layer(variable_index=int(variable_index))]
            factors.extend(
                self.mixture_layer(
                    child_layer=units.pop(int(child)),
                    weights=parameters.transition_weights(
                        parent=int(variable_index), child=int(child)
                    ),
                )
                for child in np.flatnonzero(tree.parents == variable_index)
            )
            units[int(variable_index)] = (
                factors[0]
                if len(factors) == 1
                else ProductLayer.node_wise_product_of(factors)
            )
        root_variable = int(tree.order[0])
        root = self.mixture_layer(
            child_layer=units[root_variable],
            weights=parameters.transition_weights(parent=None, child=root_variable),
        )
        return LayeredProbabilisticCircuit(
            SortedSet(self.variables), root
        ).to_namespace(jnp, floating_point_type=jnp.result_type(float))

    @staticmethod
    def mixture_layer(*, child_layer: Layer, weights: np.ndarray) -> SumLayer:
        """
        :param child_layer: The layer whose units are mixed.
        :param weights: The unnormalized weight of every unit of the child layer in
            every mixture, shape (#mixtures, #child units).
        :return: The layer of the mixtures.
        """
        rows, columns = np.nonzero(np.ones_like(weights))
        return SumLayer.from_entries(
            [child_layer],
            SparseEntries(np.log(weights[rows, columns]), rows, columns),
            weights.shape[0],
            DenseEdgeMatrix,
        )

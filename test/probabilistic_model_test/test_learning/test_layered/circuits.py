"""
Small layered circuits and datasets whose expectation maximization updates are known in
closed form, shared by the tests of learning layered circuits.
"""

from __future__ import annotations

# %% imports
from enum import IntEnum

import jax.numpy as jnp
import numpy as np
from random_events.set import Set
from random_events.variable import Continuous, Symbolic
from sortedcontainers import SortedSet

from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    SparseEntries,
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


# %% Gaussian mixture


def gaussian_mixture(
    *,
    variable: Continuous,
    weights: np.ndarray,
    locations: np.ndarray,
    scales: np.ndarray,
) -> LayeredProbabilisticCircuit:
    """
    :param variable: The variable.
    :param weights: The mixture weights.
    :param locations: The locations of the components.
    :param scales: The scales of the components.
    :return: The mixture of Gaussians as a layered circuit with jax parameters.
    """
    components = GaussianLayer(
        node_variables=np.zeros(len(locations), dtype=np.int64),
        location=locations.astype(float),
        log_scale=np.log(scales.astype(float)),
    )
    root = SumLayer.from_entries(
        [components],
        SparseEntries(
            np.log(weights.astype(float)),
            np.zeros(len(weights), dtype=np.int64),
            np.arange(len(weights)),
        ),
        1,
    )
    return LayeredProbabilisticCircuit(SortedSet([variable]), root).to_namespace(jnp)


def gaussian_mixture_responsibilities(
    *, data: np.ndarray, weights: np.ndarray, locations: np.ndarray, scales: np.ndarray
) -> np.ndarray:
    """
    :param data: The values, shape (#rows,).
    :return: The posterior probability of every component for every row.
    """
    densities = (
        weights
        * np.exp(-0.5 * ((data[:, None] - locations) / scales) ** 2)
        / (scales * np.sqrt(2 * np.pi))
    )
    return densities / densities.sum(axis=1, keepdims=True)


# %% categorical


class Colour(IntEnum):
    """
    The states of a symbolic variable with three states.
    """

    RED = 0
    GREEN = 1
    BLUE = 2


def colour_variable() -> Symbolic:
    """
    :return: A symbolic variable with three states.
    """
    return Symbolic(name="colour", domain=Set.from_iterable(Colour))


def categorical_mixture(
    *, variable: Symbolic, weights: np.ndarray, probabilities: np.ndarray
) -> LayeredProbabilisticCircuit:
    """
    :param variable: A symbolic variable.
    :param weights: The mixture weights.
    :param probabilities: The probability of every state for every component.
    :return: The mixture of categorical distributions with jax parameters.
    """
    components = SymbolicLayer(
        node_variables=np.zeros(len(probabilities), dtype=np.int64),
        states=np.arange(probabilities.shape[1]),
        table=DenseProbabilityTable(
            log_probabilities=np.log(probabilities.astype(float))
        ),
    )
    root = SumLayer.from_entries(
        [components],
        SparseEntries(
            np.log(weights.astype(float)),
            np.zeros(len(weights), dtype=np.int64),
            np.arange(len(weights)),
        ),
        1,
    )
    return LayeredProbabilisticCircuit(SortedSet([variable]), root).to_namespace(jnp)


def hashes_of_states(*, variable: Symbolic, states: np.ndarray) -> np.ndarray:
    """
    :param variable: A symbolic variable.
    :param states: State indices.
    :return: One row per state holding the hash of its domain element.
    """
    return SymbolicEncoding(variable).hashes[states][:, None]


# %% binary data with dependencies


def dependent_bits(
    *, number_of_variables: int, number_of_rows: int, seed: int
) -> tuple[SortedSet, np.ndarray]:
    """
    :param number_of_variables: An even number of variables.
    :param number_of_rows: The number of rows.
    :param seed: The seed of the rows.
    :return: Binary symbolic variables and rows of hashes, drawn from a mixture of three
        product distributions in which every odd variable copies its even neighbour with
        probability 0.9.
    """
    variables = SortedSet(
        Symbolic(name=f"x{index:02d}", domain=Set.from_iterable(["0", "1"]))
        for index in range(number_of_variables)
    )
    generator = np.random.default_rng(seed)
    component = generator.integers(0, 3, number_of_rows)
    bits = (
        generator.random((number_of_rows, number_of_variables))
        < np.array([0.15, 0.5, 0.85])[component][:, None]
    ).astype(int)
    copies = generator.random((number_of_rows, number_of_variables // 2)) < 0.9
    bits[:, 1::2] = np.where(copies, bits[:, ::2], 1 - bits[:, ::2])
    rows = np.stack(
        [
            np.array([hash("0"), hash("1")], dtype=float)[bits[:, index]]
            for index in range(number_of_variables)
        ],
        axis=1,
    )
    return variables, rows


def all_binary_rows(*, variables: SortedSet) -> np.ndarray:
    """
    :param variables: Binary symbolic variables over the domain {"0", "1"}.
    :return: Every assignment of the variables, as rows of hashes.
    """
    bits = np.array(np.meshgrid(*[[0, 1]] * len(variables), indexing="ij")).reshape(
        len(variables), -1
    )
    return np.array([hash("0"), hash("1")], dtype=float)[bits.T]

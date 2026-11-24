"""
Tests for the structure learners of layered circuits, the probabilistic dropout and the
pairwise mutual information they build on.
"""

from __future__ import annotations

# %% imports
import jax
import jax.numpy as jnp
import numpy as np
import pytest
from random_events.variable import Continuous
from scipy.special import logsumexp
from sortedcontainers import SortedSet

from probabilistic_model.learning.layered.dropout import ProbabilisticDropout
from probabilistic_model.learning.layered.exceptions import (
    UnsupportedStructureVariableError,
)
from probabilistic_model.learning.layered.expectation_maximization import (
    FullBatchExpectationMaximization,
)
from probabilistic_model.learning.layered.hidden_chow_liu_tree import (
    ChowLiuTree,
    CountedParameters,
    HiddenChowLiuTree,
)
from probabilistic_model.learning.layered.latent_variable_distillation import (
    LatentVariableDistillation,
)
from probabilistic_model.learning.layered.learn_spn import LearnSPN
from probabilistic_model.learning.layered.mutual_information import (
    DiscretizedData,
    PairwiseMutualInformation,
)
from probabilistic_model.learning.layered.pruning_and_growing import (
    DatasetEdgeFlows,
    Growing,
    Pruning,
)
from probabilistic_model.learning.layered.strudel import Strudel
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.tensorized.symbolic_encoding import (
    SymbolicEncoding,
)
from .circuits import all_binary_rows, dependent_bits


@pytest.fixture(autouse=True)
def double_precision() -> None:
    """
    Lets jax compute in double precision.
    """
    jax.config.update("jax_enable_x64", True)


@pytest.fixture
def bits() -> tuple[SortedSet, np.ndarray]:
    """
    Six binary variables with dependent pairs and 600 rows of them.
    """
    return dependent_bits(number_of_variables=6, number_of_rows=600, seed=0)


def total_probability(circuit: LayeredProbabilisticCircuit) -> float:
    """
    :param circuit: A circuit over binary variables of the domain {"0", "1"}.
    :return: The summed probability of every assignment.
    """
    rows = all_binary_rows(variables=circuit.variables)
    return float(np.exp(logsumexp(circuit.to_namespace(np).log_likelihood(events=rows))))


# %% mutual information


def test_mutual_information_of_a_copy_is_its_entropy_and_of_noise_is_small():
    """
    A column shares its entropy with its copy and nearly nothing with noise.
    """
    generator = np.random.default_rng(0)
    column = generator.integers(0, 2, 20000).astype(float)
    data = np.stack([column, column, generator.integers(0, 2, 20000)], axis=1)
    variables = SortedSet([Continuous("a"), Continuous("b"), Continuous("c")])
    mutual_information = PairwiseMutualInformation(
        discretized=DiscretizedData(variables=variables, data=data, number_of_bins=2)
    ).compute()
    frequency = column.mean()
    entropy = -(frequency * np.log(frequency) + (1 - frequency) * np.log(1 - frequency))
    assert mutual_information[0, 1] == pytest.approx(entropy)
    assert mutual_information[0, 2] < 1e-3


def test_chow_liu_tree_joins_the_most_dependent_pairs(bits):
    """
    The Chow-Liu tree connects the pairs of variables that copy each other.
    """
    variables, rows = bits
    mutual_information = PairwiseMutualInformation(
        discretized=DiscretizedData(variables=variables, data=rows)
    ).compute()
    tree = ChowLiuTree.from_mutual_information(mutual_information=mutual_information)
    edges = {
        frozenset((child, int(parent)))
        for child, parent in enumerate(tree.parents)
        if parent >= 0
    }
    assert {frozenset((0, 1)), frozenset((2, 3)), frozenset((4, 5))} <= edges
    assert len(edges) == len(variables) - 1


# %% hidden Chow-Liu trees


def test_hidden_chow_liu_tree_is_a_normalized_decomposable_circuit(bits):
    """
    A hidden Chow-Liu tree circuit sums to one over all assignments.
    """
    variables, rows = bits
    circuit = HiddenChowLiuTree(variables=variables, hidden_states=3).build_circuit(
        data=rows
    )
    assert circuit.to_namespace(np).is_decomposable()
    assert total_probability(circuit) == pytest.approx(1.0)


def test_counted_parameters_are_the_conditional_frequencies(bits):
    """
    Counted input probabilities are the frequencies of the values per latent state.
    """
    variables, rows = bits
    latent_states = np.zeros((len(rows), len(variables)), dtype=np.int64)
    # the latent state of the first variable copies the index of its value's state
    latent_states[:, 0] = SymbolicEncoding(variables[0]).indices_of_hashes(rows[:, 0])
    parameters = CountedParameters(
        variables=variables,
        data=rows,
        hidden_states=2,
        latent_states=latent_states,
        pseudocount=0.0,
    )
    np.testing.assert_allclose(
        parameters.symbolic_probabilities(variable_index=0), np.eye(2)
    )


def test_distilled_circuit_is_normalized(bits):
    """
    A distilled and fine-tuned circuit sums to one over all assignments.
    """
    variables, rows = bits
    circuit = LatentVariableDistillation(
        variables=variables,
        hidden_states=2,
        width=8,
        heads=2,
        layers=1,
        epochs=1,
        fine_tuning=FullBatchExpectationMaximization(epochs=1),
    ).build_circuit(data=rows)
    assert total_probability(circuit) == pytest.approx(1.0)


# %% pruning and growing


def test_pruning_removes_the_fraction_and_keeps_an_edge_per_unit(bits):
    """
    Pruning removes most sum edges but leaves every unit at least one.
    """
    variables, rows = bits
    circuit = HiddenChowLiuTree(variables=variables, hidden_states=4).build_circuit(
        data=rows
    )
    FullBatchExpectationMaximization(epochs=3).fit(circuit=circuit, data=rows)
    flows = DatasetEdgeFlows(root=circuit.root).compute(
        data=np.asarray(circuit.encoded(rows))
    )
    host = circuit.to_namespace(np)
    pruned = Pruning(pruned_fraction=0.75).pruned_root(root=host.root, edge_flows=flows)
    original = [layer for layer in host.layers if isinstance(layer, SumLayer)]
    remaining = [
        layer
        for layer in LayeredProbabilisticCircuit(host.variables, pruned).layers
        if isinstance(layer, SumLayer)
    ]
    original_edges = sum(layer.edges.number_of_edges for layer in original)
    remaining_edges = sum(layer.edges.number_of_edges for layer in remaining)
    assert remaining_edges < 0.5 * original_edges
    for layer in remaining:
        assert set(layer.edges.rows) == set(range(layer.number_of_nodes))


def test_growing_without_noise_keeps_the_distribution(bits):
    """
    Growing without noise doubles the units without changing the distribution.
    """
    variables, rows = bits
    circuit = (
        HiddenChowLiuTree(variables=variables, hidden_states=3)
        .build_circuit(data=rows)
        .to_namespace(np)
    )
    grown = LayeredProbabilisticCircuit(
        circuit.variables, Growing(noise_variance=0.0).grown_root(root=circuit.root)
    )
    assert grown.number_of_nodes > circuit.number_of_nodes
    np.testing.assert_allclose(grown.log_likelihood(events=rows), circuit.log_likelihood(events=rows))


# %% LearnSPN


def test_learn_spn_factorizes_independent_variables():
    """
    LearnSPN builds a product over independent variables, without mixtures.
    """
    generator = np.random.default_rng(0)
    variables = SortedSet([Continuous("a"), Continuous("b")])
    rows = generator.normal(size=(500, 2))
    circuit = LearnSPN(variables=variables).build_circuit(data=rows)
    assert not any(
        isinstance(layer, SumLayer) and layer.number_of_nodes > 1
        for layer in circuit.layers[1:]
    )
    assert circuit.is_decomposable()


def test_learn_spn_circuit_is_normalized(bits):
    """
    A LearnSPN circuit sums to one over all assignments.
    """
    variables, rows = bits
    assert total_probability(
        LearnSPN(variables=variables, minimum_rows=20).build_circuit(data=rows)
    ) == pytest.approx(1.0)


# %% Strudel


def test_strudel_splits_raise_the_training_likelihood_and_keep_determinism(bits):
    """
    Splits raise the training likelihood and keep the circuit deterministic,
    decomposable and normalized.
    """
    variables, rows = bits
    without_splits = Strudel(variables=variables, maximum_splits=0).build_circuit(
        data=rows
    )
    with_splits = Strudel(variables=variables, maximum_splits=20).build_circuit(
        data=rows
    )
    assert (
        with_splits.log_likelihood(events=rows).mean()
        > without_splits.log_likelihood(events=rows).mean()
    )
    assert with_splits.is_deterministic()
    assert with_splits.is_decomposable()
    assert total_probability(with_splits) == pytest.approx(1.0)


def test_strudel_refuses_continuous_variables():
    """
    Strudel only learns over symbolic variables.
    """
    with pytest.raises(UnsupportedStructureVariableError):
        Strudel(variables=SortedSet([Continuous("a")])).build_circuit(
            data=np.zeros((3, 1))
        )


# %% dropout


def test_dropout_without_probability_is_the_likelihood(bits):
    """
    Dropout that drops nothing is the plain likelihood.
    """
    variables, rows = bits
    circuit = HiddenChowLiuTree(variables=variables, hidden_states=3).build_circuit(
        data=rows
    )
    encoded = jnp.asarray(circuit.encoded(rows))
    np.testing.assert_allclose(
        ProbabilisticDropout().log_likelihood(
            root=circuit.root, data=encoded, key=jax.random.PRNGKey(0)
        ),
        circuit.log_likelihood(events=rows),
    )


def test_dropping_every_input_marginalizes_everything(bits):
    """
    Dropping every input distribution leaves likelihood one.
    """
    variables, rows = bits
    circuit = HiddenChowLiuTree(variables=variables, hidden_states=3).build_circuit(
        data=rows
    )
    log_likelihoods = ProbabilisticDropout(input_probability=1.0).log_likelihood(
        root=circuit.root,
        data=jnp.asarray(circuit.encoded(rows)),
        key=jax.random.PRNGKey(0),
    )
    np.testing.assert_allclose(log_likelihoods, 0.0, atol=1e-12)

"""
Tests for expectation maximization of layered circuits by tensorized expectation
maximization, its statistics adjustments and its regularizations, against updates known
in closed form.
"""

from __future__ import annotations

# %% imports
import jax
import numpy as np
import pytest
import torch
from random_events.variable import Continuous

from probabilistic_model.learning.layered.exceptions import (
    UndifferentiableBackendError,
)
from probabilistic_model.learning.layered.expectation_maximization import (
    CircuitExpectationMaximization,
)
from probabilistic_model.learning.layered.flows import TopDownProbabilities
from probabilistic_model.learning.layered.pruning_and_growing import DatasetEdgeFlows
from probabilistic_model.learning.layered.regularization import (
    DataSoftening,
    EntropyRegularization,
    NodeEntropies,
)
from probabilistic_model.learning.layered.statistics_adjustment import (
    AnemoneMixing,
    LearnedLayerStatistics,
    Momentum,
    StateCountSmoothing,
)
from probabilistic_model.learning.parameters.expectation_maximization.layer_maximization import (
    EdgeCounts,
    GaussianStatistics,
)
from probabilistic_model.probabilistic_circuit.tensorized.has_parameter_arrays import (
    HasParameterArrays,
)
from .circuits import (
    categorical_mixture,
    colour_variable,
    gaussian_mixture,
    gaussian_mixture_responsibilities,
    hashes_of_states,
)


@pytest.fixture(autouse=True)
def double_precision() -> None:
    """
    Lets jax compute in double precision, as the closed form references do.
    """
    jax.config.update("jax_enable_x64", True)


@pytest.fixture
def weights() -> np.ndarray:
    """
    The weights of the Gaussian mixture.
    """
    return np.array([0.3, 0.7])


@pytest.fixture
def locations() -> np.ndarray:
    """
    The locations of the Gaussian mixture.
    """
    return np.array([-1.0, 1.5])


@pytest.fixture
def scales() -> np.ndarray:
    """
    The scales of the Gaussian mixture.
    """
    return np.array([1.0, 0.5])


@pytest.fixture
def data() -> np.ndarray:
    """
    Values drawn from two clusters.
    """
    generator = np.random.default_rng(0)
    return np.concatenate([generator.normal(-2, 1, 60), generator.normal(2, 0.7, 40)])


def gaussian_mixture_parameters(circuit) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    :param circuit: A circuit built by :func:`gaussian_mixture`.
    :return: Its weights, locations and scales on the host.
    """
    root = circuit.root
    components = root.child_layers[0]
    return (
        HasParameterArrays.host_copy(root.normalized_weights).ravel(),
        HasParameterArrays.host_copy(components.location),
        HasParameterArrays.host_copy(components.scale),
    )


# %% flows


def test_dataset_edge_flows_are_summed_responsibilities(
    x: Continuous, weights, locations, scales, data
):
    """
    The flow of a mixture edge over a dataset is the summed posterior of its component.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    flows = DatasetEdgeFlows(circuit=circuit, batch_size=32).compute(data=data[:, None])
    responsibilities = gaussian_mixture_responsibilities(
        data=data, weights=weights, locations=locations, scales=scales
    )
    np.testing.assert_allclose(flows[0], responsibilities.sum(axis=0))


def test_top_down_probabilities_of_a_mixture_are_its_weights(
    x: Continuous, weights, locations, scales
):
    """
    Top-down sampling reaches a mixture component with its weight.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    top_down = TopDownProbabilities(root=circuit.root).compute()
    np.testing.assert_allclose(
        top_down.node_values[circuit.root.child_layers[0]], weights
    )


def test_numpy_parameters_cannot_be_learned(x: Continuous, weights, locations, scales):
    """
    A circuit with numpy parameters has no gradient to count statistics from.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    ).to_namespace(np)
    with pytest.raises(UndifferentiableBackendError):
        CircuitExpectationMaximization().learner(root=circuit.root)


# %% expectation maximization


def test_full_batch_epoch_is_the_closed_form_mixture_update(
    x: Continuous, weights, locations, scales, data
):
    """
    One full-batch epoch is the textbook expectation maximization step of a Gaussian
    mixture.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    CircuitExpectationMaximization(epochs=1).fit(circuit=circuit, data=data[:, None])
    responsibilities = gaussian_mixture_responsibilities(
        data=data, weights=weights, locations=locations, scales=scales
    )
    counts = responsibilities.sum(axis=0)
    expected_locations = (responsibilities * data[:, None]).sum(axis=0) / counts
    expected_scales = np.sqrt(
        (responsibilities * (data[:, None] - expected_locations) ** 2).sum(axis=0)
        / counts
    )
    new_weights, new_locations, new_scales = gaussian_mixture_parameters(circuit)
    np.testing.assert_allclose(new_weights, counts / len(data))
    np.testing.assert_allclose(new_locations, expected_locations)
    np.testing.assert_allclose(new_scales, expected_scales)


def test_categorical_probabilities_are_smoothed_counts():
    """
    A categorical node becomes its counts plus the pseudocount, normalized, so a state
    no row shows keeps a positive probability.
    """
    variable = colour_variable()
    circuit = categorical_mixture(
        variable=variable,
        weights=np.array([1.0]),
        probabilities=np.array([[0.2, 0.3, 0.5]]),
    )
    states = np.array([0, 0, 0, 1, 1])
    CircuitExpectationMaximization(epochs=1).fit(
        circuit=circuit, data=hashes_of_states(variable=variable, states=states)
    )
    counts = np.bincount(states, minlength=3) + StateCountSmoothing().pseudocount
    np.testing.assert_allclose(
        np.exp(
            HasParameterArrays.host_copy(
                circuit.root.child_layers[0].table.log_probabilities
            )
        ).ravel(),
        counts / counts.sum(),
    )


def test_anemone_step_mixes_by_top_down_probability_and_flow(
    x: Continuous, weights, locations, scales, data
):
    """
    Anemone mixes the old weights by top-down probability and the batch by flow,
    equation 7.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    mixing = AnemoneMixing()
    CircuitExpectationMaximization(
        epochs=1,
        batch_size=len(data),
        batches_per_update=1,
        statistics_adjustments=[mixing],
    ).fit(circuit=circuit, data=data[:, None])
    average_counts = gaussian_mixture_responsibilities(
        data=data, weights=weights, locations=locations, scales=scales
    ).sum(axis=0) / len(data)
    # the root has top-down probability one and flow one per row
    expected = (weights + mixing.learning_rate * average_counts) / (
        1 + mixing.learning_rate
    )
    np.testing.assert_allclose(gaussian_mixture_parameters(circuit)[0], expected)


def test_anemone_with_torch_equals_anemone_with_jax(
    x: Continuous, weights, locations, scales, data
):
    """
    The adjusted update gives the same parameters on both differentiating backends.
    """
    jax_circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    torch_circuit = jax_circuit.to_namespace(torch)
    anemone = CircuitExpectationMaximization.anemone(epochs=2)
    anemone.fit(circuit=jax_circuit, data=data[:, None])
    anemone.fit(circuit=torch_circuit, data=data[:, None])
    for torch_parameter, jax_parameter in zip(
        gaussian_mixture_parameters(torch_circuit),
        gaussian_mixture_parameters(jax_circuit),
    ):
        np.testing.assert_allclose(torch_parameter, jax_parameter)


def test_momentum_keeps_the_first_statistics_and_averages_the_next(
    x: Continuous, weights, locations, scales
):
    """
    The bias-corrected average of one update is that update; the second is the bias-
    corrected mean of both updates' statistics per row.
    """
    root = (
        gaussian_mixture(
            variable=x, weights=weights, locations=locations, scales=scales
        )
        .to_namespace(np)
        .root
    )
    momentum = Momentum(decay=0.5)
    first = EdgeCounts(counts=np.array([[2.0, 6.0]]))
    second = EdgeCounts(counts=np.array([[3.0, 1.0]]))
    adjusted_first = momentum.adjusted(
        statistics=LearnedLayerStatistics(
            root=root, layers=[root], statistics=[first], number_of_rows=8
        )
    )
    adjusted_second = momentum.adjusted(
        statistics=LearnedLayerStatistics(
            root=root, layers=[root], statistics=[second], number_of_rows=4
        )
    )
    np.testing.assert_allclose(adjusted_first.statistics[0].counts, first.counts)
    # averages per row: 0.25 * first / 8 + 0.5 * second / 4, corrected by 1 - 0.25
    expected_per_row = (0.25 * first.counts / 8 + 0.5 * second.counts / 4) / 0.75
    np.testing.assert_allclose(
        adjusted_second.statistics[0].counts, expected_per_row * 4
    )


def test_momentum_averages_gaussian_sums_about_zero(
    x: Continuous, weights, locations, scales
):
    """
    Centering the averaged Gaussian sums at the layer's location again gives the
    statistics back on the first update.
    """
    components = (
        gaussian_mixture(
            variable=x, weights=weights, locations=locations, scales=scales
        )
        .to_namespace(np)
        .root.child_layers[0]
    )
    statistics = GaussianStatistics(
        counts=np.array([4.0, 6.0]),
        centered_sums=np.array([1.0, -2.0]),
        centered_squared_sums=np.array([5.0, 3.0]),
    )
    adjusted = Momentum(decay=0.9).adjusted(
        statistics=LearnedLayerStatistics(
            root=components,
            layers=[components],
            statistics=[statistics],
            number_of_rows=10,
        )
    )
    for adjusted_array, array in zip(adjusted.statistics[0].arrays, statistics.arrays):
        np.testing.assert_allclose(adjusted_array, array)


# %% regularization


def test_softened_counts_move_mass_to_the_other_states():
    """
    Softened data counts every value partly as each other state.
    """
    variable = colour_variable()
    states = np.array([0, 0, 0, 1])
    keep_probability = 0.9
    circuit = categorical_mixture(
        variable=variable,
        weights=np.array([1.0]),
        probabilities=np.array([[1 / 3, 1 / 3, 1 / 3]]),
    )
    CircuitExpectationMaximization(
        epochs=1,
        data_softening=DataSoftening(keep_probability=keep_probability),
        statistics_adjustments=[],
    ).fit(circuit=circuit, data=hashes_of_states(variable=variable, states=states))
    frequencies = np.bincount(states, minlength=3) / len(states)
    # every row keeps its state with the keep probability and moves to each of the two
    # others with half the rest; the uniform start makes the posterior that prior
    expected = (
        keep_probability * frequencies + (1 - keep_probability) * (1 - frequencies) / 2
    )
    np.testing.assert_allclose(
        np.exp(
            HasParameterArrays.host_copy(
                circuit.root.child_layers[0].table.log_probabilities
            )
        ).ravel(),
        expected,
    )


def test_entropy_regularization_without_strength_is_the_plain_update(
    x: Continuous, weights, locations, scales, data
):
    """
    Without entropy pull, the regularized update is the plain full-batch update.
    """
    plain = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    regularized = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    CircuitExpectationMaximization(epochs=1).fit(circuit=plain, data=data[:, None])
    CircuitExpectationMaximization(
        epochs=1,
        statistics_adjustments=[
            StateCountSmoothing(),
            EntropyRegularization(strength=0.0),
        ],
    ).fit(circuit=regularized, data=data[:, None])
    for regularized_parameter, plain_parameter in zip(
        gaussian_mixture_parameters(regularized), gaussian_mixture_parameters(plain)
    ):
        np.testing.assert_allclose(regularized_parameter, plain_parameter)


def test_entropy_regularization_without_strength_normalizes_the_counts():
    """
    Without entropy pull, the regularized weights are the normalized counts.
    """
    counts = np.array([3.0, 1.0, 2.0, 2.0])
    owners = np.array([0, 0, 1, 1])
    weights = EntropyRegularization(strength=0.0).regularized_weights(
        average_counts=counts,
        owners=owners,
        top_down_probabilities=np.ones(2),
        child_entropies=np.zeros(4),
    )
    np.testing.assert_allclose(weights, [0.75, 0.25, 0.5, 0.5])


def test_entropy_regularized_weights_solve_the_stationarity_equations():
    """
    The regularized weights satisfy equation 5 of the paper and sum to one.
    """
    counts = np.array([0.6, 0.3, 0.1])
    owners = np.zeros(3, dtype=np.int64)
    child_entropies = np.array([0.5, 1.0, 2.0])
    regularization = EntropyRegularization(strength=0.2)
    weights = regularization.regularized_weights(
        average_counts=counts,
        owners=owners,
        top_down_probabilities=np.ones(1),
        child_entropies=child_entropies,
    )
    pull = regularization.strength
    left_sides = counts / weights - pull * np.log(weights) + pull * child_entropies
    assert weights.sum() == pytest.approx(1.0)
    np.testing.assert_allclose(left_sides, left_sides[0], rtol=1e-6)


def test_entropy_of_a_gaussian_mixture_over_its_latent_variable(
    x: Continuous, weights, locations, scales
):
    """
    The entropy of a mixture over its latent variable adds the component entropies.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    entropies = NodeEntropies(root=circuit.root).compute()
    component_entropies = 0.5 * np.log(2 * np.pi * np.e * scales**2)
    assert entropies[circuit.root][0] == pytest.approx(
        np.sum(weights * (component_entropies - np.log(weights)))
    )

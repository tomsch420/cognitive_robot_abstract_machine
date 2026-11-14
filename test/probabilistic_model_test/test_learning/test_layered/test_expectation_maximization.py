"""
Tests for the expected flows, the expectation maximization variants and their
regularizations on layered circuits, against updates known in closed form.
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
    Anemone,
    FullBatchExpectationMaximization,
    MiniBatchExpectationMaximization,
)
from probabilistic_model.learning.layered.flows import (
    ExpectedFlows,
    TopDownProbabilities,
)
from probabilistic_model.learning.layered.gradients import LogLikelihoodGradient
from probabilistic_model.learning.layered.maximization import (
    CategoricalLayerMaximization,
)
from probabilistic_model.learning.layered.regularization import (
    DataSoftening,
    EntropyRegularization,
    NodeEntropies,
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


def test_expected_flows_are_summed_responsibilities(
    x: Continuous, weights, locations, scales, data
):
    """
    The flow of a mixture edge is the summed posterior of its component.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    result = LogLikelihoodGradient.from_root(root=circuit.root).compute(
        data=jax.numpy.asarray(data[:, None])
    )
    flows = ExpectedFlows.from_parameter_gradients(
        root=circuit.root, gradients=result.gradients, number_of_rows=len(data)
    ).compute()
    responsibilities = gaussian_mixture_responsibilities(
        data=data, weights=weights, locations=locations, scales=scales
    )
    np.testing.assert_allclose(
        flows.edge_values[circuit.root], responsibilities.sum(axis=0)
    )
    np.testing.assert_allclose(
        flows.node_values[circuit.root.child_layers[0]], responsibilities.sum(axis=0)
    )
    assert flows.node_values[circuit.root][0] == len(data)


def test_torch_flows_equal_jax_flows(x: Continuous, weights, locations, scales, data):
    """
    Both differentiating backends give the same likelihood and gradient.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    torch_circuit = circuit.to_namespace(torch)
    jax_result = LogLikelihoodGradient.from_root(root=circuit.root).compute(
        data=jax.numpy.asarray(data[:, None])
    )
    torch_result = LogLikelihoodGradient.from_root(root=torch_circuit.root).compute(
        data=torch.as_tensor(data[:, None])
    )
    assert torch_result.log_likelihood == pytest.approx(jax_result.log_likelihood)
    for torch_gradient, jax_gradient in zip(
        torch_result.gradients, jax_result.gradients
    ):
        np.testing.assert_allclose(torch_gradient, jax_gradient)


def test_numpy_parameters_cannot_be_differentiated(
    x: Continuous, weights, locations, scales
):
    """
    A circuit with numpy parameters has no gradient to read flows from.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    ).to_namespace(np)
    with pytest.raises(UndifferentiableBackendError):
        LogLikelihoodGradient.from_root(root=circuit.root)


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


# %% expectation maximization


def test_full_batch_step_is_the_closed_form_mixture_update(
    x: Continuous, weights, locations, scales, data
):
    """
    One full-batch step is the textbook expectation maximization step of a Gaussian
    mixture.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    FullBatchExpectationMaximization(epochs=1).fit(circuit=circuit, data=data[:, None])
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


def test_full_batch_expectation_maximization_never_lowers_the_likelihood(
    x: Continuous, weights, locations, scales, data
):
    """
    Every full-batch step keeps or raises the training likelihood.
    """
    circuit = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    method = FullBatchExpectationMaximization(epochs=10)
    method.fit(circuit=circuit, data=data[:, None])
    assert np.all(np.diff(method.average_log_likelihoods) >= -1e-12)


def test_mini_batch_step_of_full_size_one_is_the_full_batch_step(
    x: Continuous, weights, locations, scales, data
):
    """
    A mini-batch step over all rows with step size one is the full-batch step.
    """
    full = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    mini = gaussian_mixture(
        variable=x, weights=weights, locations=locations, scales=scales
    )
    FullBatchExpectationMaximization(epochs=1).fit(circuit=full, data=data[:, None])
    MiniBatchExpectationMaximization(epochs=1, batch_size=len(data), step_size=1.0).fit(
        circuit=mini, data=data[:, None]
    )
    for full_parameter, mini_parameter in zip(
        gaussian_mixture_parameters(full), gaussian_mixture_parameters(mini)
    ):
        np.testing.assert_allclose(mini_parameter, full_parameter)


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
    method = Anemone(epochs=1, batch_size=len(data), momentum=0.0)
    method.fit(circuit=circuit, data=data[:, None])
    average_counts = gaussian_mixture_responsibilities(
        data=data, weights=weights, locations=locations, scales=scales
    ).sum(axis=0) / len(data)
    # the root has top-down probability one and flow one per row
    expected = (weights + method.learning_rate * average_counts) / (
        1 + method.learning_rate
    )
    np.testing.assert_allclose(gaussian_mixture_parameters(circuit)[0], expected)


def test_categorical_probabilities_are_smoothed_counts():
    """
    A categorical node becomes its counts plus the pseudocount, normalized.
    """
    variable = colour_variable()
    circuit = categorical_mixture(
        variable=variable,
        weights=np.array([1.0]),
        probabilities=np.array([[0.2, 0.3, 0.5]]),
    )
    states = np.array([0, 0, 0, 1, 2, 2])
    pseudocount = CategoricalLayerMaximization().pseudocount
    FullBatchExpectationMaximization(epochs=1).fit(
        circuit=circuit, data=hashes_of_states(variable=variable, states=states)
    )
    counts = np.bincount(states, minlength=3) + pseudocount
    np.testing.assert_allclose(
        np.exp(
            HasParameterArrays.host_copy(
                circuit.root.child_layers[0].table.log_probabilities
            )
        ).ravel(),
        counts / counts.sum(),
    )


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
    method = FullBatchExpectationMaximization(
        epochs=1, data_softening=DataSoftening(keep_probability=keep_probability)
    )
    method.maximizations.maximizations[1].pseudocount = 0.0
    method.fit(circuit=circuit, data=hashes_of_states(variable=variable, states=states))
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

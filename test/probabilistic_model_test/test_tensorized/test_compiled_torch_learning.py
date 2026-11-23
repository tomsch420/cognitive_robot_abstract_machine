"""
Tests that torch learners whose steps are compiled learn what they learn operation by
operation, and that a compiled step is one graph.
"""

from __future__ import annotations

# %% imports
from collections.abc import Iterator

import numpy as np
import pytest
from array_api_compat import array_namespace

from probabilistic_model.learning.gradient_descent import AverageNegativeLogLikelihood
from probabilistic_model.learning.training_schedule import Convergence, MiniBatches
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.edge_matrix import (
    BlockEdgeMatrix,
    EdgeMatrix,
)
from .test_inner_layers_on_array_backends import MixtureOfProductsCase

torch = pytest.importorskip("torch")
torch_expectation_maximization = pytest.importorskip(
    "probabilistic_model.learning.torch_expectation_maximization"
)
torch_gradient_descent = pytest.importorskip(
    "probabilistic_model.learning.torch_gradient_descent"
)

# %% fixtures


@pytest.fixture(autouse=True)
def fresh_compilation() -> Iterator[None]:
    """
    Forget every compiled function and graph break before and after the test, so that it
    compiles from scratch and leaves no compiled functions to other tests.
    """
    torch._dynamo.reset()
    torch._dynamo.utils.counters.clear()
    yield
    torch._dynamo.reset()


def parameters_after_learning(
    *,
    case: MixtureOfProductsCase,
    edge_matrix_type: type[EdgeMatrix],
    device: str,
    learner_type: type,
    **settings: object,
) -> list[np.ndarray]:
    """
    :param case: The circuit to learn and to sample the data from.
    :param edge_matrix_type: The type of matrix every inner layer stores its edges in.
    :param device: The torch device to learn on.
    :param learner_type: The torch learner.
    :param settings: Further fields of the learner.
    :return: The parameters after three epochs on mini-batches of 100 of 450 rows the
        circuit samples, so that the batches have two shapes.
    """
    np.random.seed(0)
    host = case.circuit(np, edge_matrix_type)
    namespace = array_namespace(torch.zeros(0))
    circuit = host.to_namespace(namespace, device=device)
    run = learner_type(layer=circuit.root, **settings).minimize_until_converged(
        namespace.asarray(host.sample(450), device=device),
        convergence=Convergence(maximum_epochs=3, patience=3),
        mini_batches=MiniBatches(batch_size=100, seed=0),
    )
    return [
        parameter.detach().cpu().numpy()
        for parameter in run.trained_root.circuit_parameters
    ]


def online_expectation_maximization_settings() -> dict[str, object]:
    """
    :return: The fields of an online expectation maximization that updates after every
        batch by half the way.
    """
    return dict(
        learner_type=torch_expectation_maximization.TorchExpectationMaximization,
        batches_per_update=1,
        step_size=0.5,
    )


def gradient_descent_settings() -> dict[str, object]:
    """
    :return: The fields of a gradient descent on the average negative log-likelihood.
    """
    return dict(
        learner_type=torch_gradient_descent.TorchGradientDescent,
        loss=AverageNegativeLogLikelihood(),
        learning_rate=0.05,
    )


# %% compiled and operation by operation


@pytest.mark.parametrize(
    "settings",
    [online_expectation_maximization_settings(), gradient_descent_settings()],
    ids=["expectation-maximization", "gradient-descent"],
)
def test_compiled_steps_learn_what_steps_operation_by_operation_learn(
    mixture_of_products_case, settings
):
    """
    Compiling the steps changes none of the parameters learned on the CPU beyond the
    rounding of double precision.
    """
    compiled, operation_by_operation = (
        parameters_after_learning(
            case=mixture_of_products_case,
            edge_matrix_type=BlockEdgeMatrix,
            device="cpu",
            compiles_steps=compiles_steps,
            **settings,
        )
        for compiles_steps in (True, False)
    )
    for compiled_parameter, parameter in zip(
        compiled, operation_by_operation, strict=True
    ):
        np.testing.assert_allclose(compiled_parameter, parameter, rtol=1e-9, atol=1e-8)


@pytest.mark.parametrize(
    "settings",
    [online_expectation_maximization_settings(), gradient_descent_settings()],
    ids=["expectation-maximization", "gradient-descent"],
)
def test_compiled_steps_recorded_as_cuda_graphs_learn_what_recorded_steps_learn(
    mixture_of_products_case, settings
):
    """
    Compiling the steps that are recorded as CUDA graphs and replayed changes none of
    the parameters learned beyond the rounding of double precision.
    """
    if not torch.cuda.is_available():
        pytest.skip("needs a CUDA device")
    compiled, operation_by_operation = (
        parameters_after_learning(
            case=mixture_of_products_case,
            edge_matrix_type=BlockEdgeMatrix,
            device="cuda",
            compiles_steps=compiles_steps,
            records_cuda_graphs=True,
            **settings,
        )
        for compiles_steps in (True, False)
    )
    for compiled_parameter, parameter in zip(
        compiled, operation_by_operation, strict=True
    ):
        np.testing.assert_allclose(compiled_parameter, parameter, rtol=1e-9, atol=1e-8)


# %% one graph


def test_a_compiled_step_is_one_graph_for_every_shape_of_the_batches(
    mixture_of_products_case, edge_matrix_type
):
    """
    The compiled step on a circuit of every type of edge matrix is one graph for each of
    the two shapes of the batches.
    """
    parameters_after_learning(
        case=mixture_of_products_case,
        edge_matrix_type=edge_matrix_type,
        device="cpu",
        compiles_steps=True,
        **online_expectation_maximization_settings(),
    )
    assert torch._dynamo.utils.counters["stats"]["unique_graphs"] == 2

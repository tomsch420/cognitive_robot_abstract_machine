"""
Tests of the learning methods of the circuit learning benchmark: the settings they learn
in and the circuits they learn in them.
"""

from __future__ import annotations

# %% imports
import dataclasses
import jax
import numpy as np
import pandas as pd
import pytest
from krrood_compute_resources.devices import Device

from experiments.circuit_learning.datasets import BenchmarkDataset, DatasetName
from experiments.circuit_learning.methods import (
    BenchmarkMethod,
    BenchmarkMethods,
    CircuitExpectationMaximizationMethod,
    HiddenChowLiuTreeMethod,
    LatentVariableDistillationMethod,
    LearningMethod,
    LearnSPNMethod,
    PruningAndGrowingMethod,
    RandomTensorizedSumProductNetworkByAnemone,
    RandomTensorizedSumProductNetworkByDropout,
    RandomTensorizedSumProductNetworkByExpectationMaximization,
    RandomTensorizedSumProductNetworkByOnlineExpectationMaximization,
    StrudelMethod,
    TrainingContext,
)
from experiments.circuit_learning.settings import (
    ArrayLibrary,
    ComputationSetting,
    EdgeRepresentation,
    FloatingPointType,
)
from probabilistic_model.learning.parameters.training_schedule import Convergence

# %% settings


def test_expectation_maximization_learns_on_the_host_and_in_every_setting_of_the_grid():
    """
    Full-batch expectation maximization runs on the host and in every jax and torch
    setting; the online one only where it is tensorized.
    """
    grid = ComputationSetting.grid(
        array_libraries=(ArrayLibrary.JAX, ArrayLibrary.TORCH)
    )

    assert RandomTensorizedSumProductNetworkByExpectationMaximization().settings() == [
        ComputationSetting.host(),
        *grid,
    ]
    assert (
        RandomTensorizedSumProductNetworkByOnlineExpectationMaximization().settings()
        == grid
    )


# %% learning


def test_tensorized_expectation_maximization_learns_what_the_host_one_learns(dataset):
    """
    Two epochs of full-batch expectation maximization with jax in double precision give
    every row the log-likelihood that two iterations on the host give it.
    """
    method = RandomTensorizedSumProductNetworkByExpectationMaximization()
    context = TrainingContext(
        convergence=Convergence(maximum_epochs=2, patience=2),
        memory_budget_bytes=2**30,
    )
    jax_setting = ComputationSetting(
        array_library=ArrayLibrary.JAX,
        device=Device.CENTRAL_PROCESSOR,
        floating_point_type=FloatingPointType.FLOAT64,
        edge_representation=EdgeRepresentation.BLOCK,
    )
    rows = dataset.encoded(frame=dataset.train)
    with jax.enable_x64(True):
        tensorized = method.learn(
            dataset=dataset, setting=jax_setting, context=context
        ).circuit
        tensorized_log_likelihoods = np.asarray(
            tensorized.log_likelihood(events=jax_setting.asarray(rows=rows))
        )
    host = method.learn(
        dataset=dataset, setting=ComputationSetting.host(), context=context
    ).circuit

    np.testing.assert_allclose(
        tensorized_log_likelihoods, host.log_likelihood(events=rows), rtol=1e-8
    )


# %% ported methods


@pytest.fixture
def symbolic_dataset() -> BenchmarkDataset:
    """
    :return: Three symbolic columns, the second a copy of the first.
    """
    generator = np.random.default_rng(0)

    def frame(number_of_rows: int) -> pd.DataFrame:
        """
        :param number_of_rows: The number of rows.
        :return: Rows of the three columns.
        """
        first = generator.integers(0, 2, number_of_rows)
        return pd.DataFrame(
            {
                "a": np.where(first == 1, "yes", "no"),
                "b": np.where(first == 1, "yes", "no"),
                "c": np.where(generator.random(number_of_rows) < 0.3, "on", "off"),
            }
        )

    return BenchmarkDataset(
        name=DatasetName.NLTCS, train=frame(200), validation=frame(50), test=frame(50)
    )


@pytest.fixture
def short_training() -> TrainingContext:
    """
    :return: Training for two epochs per phase in a gibibyte.
    """
    return TrainingContext(
        convergence=Convergence(maximum_epochs=2, patience=2),
        memory_budget_bytes=2**30,
    )


def jax_central_processor_setting() -> ComputationSetting:
    """
    :return: Jax on the CPU in single precision with block edges, the reference setting
        on the device every test machine has.
    """
    return ComputationSetting(
        array_library=ArrayLibrary.JAX,
        device=Device.CENTRAL_PROCESSOR,
        floating_point_type=FloatingPointType.FLOAT32,
        edge_representation=EdgeRepresentation.BLOCK,
    )


def test_every_method_has_a_name_of_its_own():
    """
    No two methods of the benchmark are reported under the same name.
    """
    names = [method.name for method in BenchmarkMethods().methods]
    assert len(names) == len(set(names))
    assert set(names) == set(LearningMethod)


def test_dropout_learns_with_jax_alone():
    """
    Gradient descent under dropout runs in every jax setting and in no other.
    """
    assert RandomTensorizedSumProductNetworkByDropout().settings() == (
        ComputationSetting.grid(array_libraries=(ArrayLibrary.JAX,))
    )


def test_strudel_supports_symbolic_datasets_alone(dataset, symbolic_dataset):
    """
    Strudel learns from datasets whose variables are all symbolic.
    """
    assert not StrudelMethod().supports(dataset=dataset)
    assert StrudelMethod().supports(dataset=symbolic_dataset)


@pytest.mark.parametrize(
    "method",
    [
        RandomTensorizedSumProductNetworkByAnemone(),
        HiddenChowLiuTreeMethod(),
        BenchmarkMethods().find(name=LearningMethod.REGULARIZED_HIDDEN_CHOW_LIU_TREE),
        BenchmarkMethods().find(name=LearningMethod.HIDDEN_CHOW_LIU_TREE_ANEMONE),
        LatentVariableDistillationMethod(),
    ],
    ids=lambda method: method.name.value,
)
def test_expectation_maximization_methods_train_every_phase(
    method: CircuitExpectationMaximizationMethod, dataset, short_training
):
    """
    A method of phases of expectation maximization trains each phase as long as the
    context says and learns a circuit that gives every test row a likelihood.
    """
    setting = jax_central_processor_setting()
    learned = method.learn(dataset=dataset, setting=setting, context=short_training)
    log_likelihoods = np.asarray(
        learned.circuit.log_likelihood(events=dataset.encoded(frame=dataset.test))
    )
    assert len(learned.history.epoch_losses) == 2 * len(method.phases)
    assert np.isfinite(log_likelihoods).all()


def test_adjusted_expectation_maximization_trains_with_torch(dataset, short_training):
    """
    The regularized hidden Chow-Liu tree, whose updates run on the host, trains with
    torch too.
    """
    method = BenchmarkMethods().find(
        name=LearningMethod.REGULARIZED_HIDDEN_CHOW_LIU_TREE
    )
    setting = dataclasses.replace(
        jax_central_processor_setting(), array_library=ArrayLibrary.TORCH
    )
    learned = method.learn(dataset=dataset, setting=setting, context=short_training)
    log_likelihoods = learned.circuit.log_likelihood(
        events=dataset.encoded(frame=dataset.test)
    )
    assert np.isfinite(log_likelihoods.cpu().numpy()).all()


@pytest.mark.parametrize(
    "method",
    [RandomTensorizedSumProductNetworkByDropout(), PruningAndGrowingMethod()],
    ids=lambda method: method.name.value,
)
def test_gradient_descent_and_structure_search_learn_in_jax(
    method: BenchmarkMethod, dataset, short_training
):
    """
    Dropout and pruning and growing learn a circuit in jax that gives every test row a
    likelihood.
    """
    setting = jax_central_processor_setting()
    learned = method.learn(dataset=dataset, setting=setting, context=short_training)
    log_likelihoods = np.asarray(
        learned.circuit.log_likelihood(events=dataset.encoded(frame=dataset.test))
    )
    assert np.isfinite(log_likelihoods).all()


@pytest.mark.parametrize(
    "method", [LearnSPNMethod(), StrudelMethod()], ids=lambda method: method.name.value
)
def test_structure_learners_learn_on_the_host(
    method: BenchmarkMethod, symbolic_dataset, short_training
):
    """
    LearnSPN and Strudel learn a circuit on the host that gives every test row a
    likelihood.
    """
    learned = method.learn(
        dataset=symbolic_dataset,
        setting=ComputationSetting.host(),
        context=short_training,
    )
    log_likelihoods = learned.circuit.log_likelihood(
        events=symbolic_dataset.encoded(frame=symbolic_dataset.test)
    )
    assert learned.history is None
    assert np.isfinite(log_likelihoods).all()

"""
Tests of the cirkit circuits that compute what a layered RAT-SPN computes, and of the
benchmark method that trains RAT-SPNs with cirkit.
"""

from __future__ import annotations

# %% imports
from datetime import timedelta

import numpy as np
import pandas as pd
import pytest
import torch
from random_events.set import Set
from random_events.variable import Continuous, Symbolic
from sortedcontainers import SortedSet

from experiments.circuit_learning.cirkit_comparison.circuits import (
    CirkitCircuit,
    CirkitQueryMeasurement,
    ConvertedGaussianLayer,
    CirkitTraining,
)
from experiments.circuit_learning.cirkit_comparison.exceptions import (
    LayerWithoutCirkitCounterpartError,
)
from experiments.circuit_learning.datasets import BenchmarkDataset, DatasetName
from experiments.circuit_learning.methods import (
    RandomTensorizedSumProductNetworkByCirkit,
    TrainingContext,
)
from experiments.circuit_learning.rows import (
    BenchmarkConfiguration,
    LikelihoodSpeedResult,
)
from experiments.circuit_learning.settings import ComputationSetting
from experiments.experiment_definitions import ExperimentResult
from probabilistic_model.learning.region_graph.region_graph import RegionGraph
from probabilistic_model.learning.training_schedule import Convergence, MiniBatches
from probabilistic_model.probabilistic_circuit.tensorized.input_layer.gaussian_layer import (
    GaussianLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
    LayerNode,
)

# %% fixtures


@pytest.fixture
def mixed_variables() -> SortedSet:
    """
    :return: Four continuous and two symbolic variables.
    """
    return SortedSet(
        [Continuous(f"x{index}") for index in range(4)]
        + [
            Symbolic(name="colour", domain=Set.from_iterable(["red", "green", "blue"])),
            Symbolic(name="size", domain=Set.from_iterable(["small", "large"])),
        ]
    )


@pytest.fixture
def random_circuit(mixed_variables: SortedSet) -> LayeredProbabilisticCircuit:
    """
    :return: A randomly initialized RAT-SPN over the mixed variables.
    """
    return (
        RegionGraph(variables=mixed_variables, depth=2, repetitions=3)
        .create_random_region_graph()
        .to_probabilistic_circuit(input_units=3, sum_units=3)
    )


@pytest.fixture
def encoded_rows(mixed_variables: SortedSet) -> np.ndarray:
    """
    :return: Rows over the mixed variables, every symbolic value the position of its
        element in the domain.
    """
    generator = np.random.default_rng(0)
    columns = [
        (
            generator.integers(len(variable.domain.simple_sets), size=40).astype(float)
            if isinstance(variable, Symbolic)
            else generator.normal(size=40)
        )
        for variable in mixed_variables
    ]
    return np.stack(columns, axis=1)


@pytest.fixture
def mixed_dataset() -> BenchmarkDataset:
    """
    :return: A small dataset of three continuous and one symbolic column.
    """
    generator = np.random.default_rng(1)

    def split(rows: int) -> pd.DataFrame:
        return pd.DataFrame(
            {
                "a": generator.normal(size=rows),
                "b": generator.normal(size=rows),
                "c": generator.normal(size=rows),
                "d": generator.choice(["u", "v"], size=rows),
            }
        )

    return BenchmarkDataset(
        name=DatasetName.NLTCS, train=split(64), validation=split(16), test=split(16)
    )


def log_likelihoods_of(*, circuit: CirkitCircuit, rows: np.ndarray) -> np.ndarray:
    """
    :return: The log-likelihoods the compiled cirkit circuit gives the encoded rows.
    """
    with torch.no_grad():
        return (
            circuit.log_likelihood(rows=torch.asarray(rows, dtype=torch.float64))
            .cpu()
            .numpy()
        )


# %% conversion


def test_cirkit_circuit_of_a_random_network_has_its_log_likelihoods(
    random_circuit: LayeredProbabilisticCircuit, encoded_rows: np.ndarray
):
    """
    The cirkit circuit of a RAT-SPN gives every row the log-likelihood the layered
    circuit gives it.
    """
    circuit = CirkitCircuit(
        layered=random_circuit, device=torch.device("cpu"), dtype=torch.float64
    )
    np.testing.assert_allclose(
        log_likelihoods_of(circuit=circuit, rows=encoded_rows),
        random_circuit.root.log_likelihood_of_nodes(encoded_rows)[:, 0],
        rtol=1e-6,
    )


def test_trained_layered_circuit_has_the_log_likelihoods_of_the_trained_cirkit_circuit(
    random_circuit: LayeredProbabilisticCircuit, encoded_rows: np.ndarray
):
    """
    After cirkit trains its circuit, the layered circuit read back from it gives every
    row the log-likelihood the trained cirkit circuit gives it.
    """
    circuit = CirkitCircuit(
        layered=random_circuit, device=torch.device("cpu"), dtype=torch.float64
    )
    training = CirkitTraining(
        arguments=(torch.asarray(encoded_rows, dtype=torch.float64),),
        mini_batches=MiniBatches(),
        circuit=circuit,
        learning_rate=0.1,
    )
    training.train_one_epoch()
    trained = circuit.trained_layered()
    cirkit_log_likelihoods = log_likelihoods_of(circuit=circuit, rows=encoded_rows)
    assert not np.allclose(
        cirkit_log_likelihoods,
        random_circuit.root.log_likelihood_of_nodes(encoded_rows)[:, 0],
    )
    np.testing.assert_allclose(
        trained.root.log_likelihood_of_nodes(encoded_rows)[:, 0],
        cirkit_log_likelihoods,
        rtol=1e-6,
    )


def test_cirkit_training_keeps_every_scale_at_least_the_minimum_scale(
    random_circuit: LayeredProbabilisticCircuit, encoded_rows: np.ndarray
):
    """
    Training on a continuous column of one value, which a vanishing scale fits ever
    better, keeps every standard deviation of the compiled cirkit circuit at least the
    minimum scale of the layered circuit.
    """
    rows = encoded_rows.copy()
    rows[:, 0] = 0.5
    circuit = CirkitCircuit(
        layered=random_circuit, device=torch.device("cpu"), dtype=torch.float64
    )
    training = CirkitTraining(
        arguments=(torch.asarray(rows, dtype=torch.float64),),
        mini_batches=MiniBatches(),
        circuit=circuit,
        learning_rate=10.0,
    )
    training.train_one_epoch()
    standard_deviations = np.concatenate(
        [
            np.exp(circuit.tensor_values(tensor=converted.log_standard_deviation))
            for converted in circuit.converted_layers.values()
            if isinstance(converted, ConvertedGaussianLayer)
        ]
    )
    assert standard_deviations.min() >= GaussianLayer.minimum_scale


def test_cirkit_circuit_over_a_variable_with_one_value_has_its_log_likelihoods():
    """
    A symbolic variable with a single value, which cirkit's categorical layers do not
    take, keeps the log-likelihoods of the layered circuit.
    """
    variables = SortedSet(
        [
            Continuous("x"),
            Symbolic(name="constant", domain=Set.from_iterable(["only"])),
        ]
    )
    layered = (
        RegionGraph(variables=variables, depth=1, repetitions=2)
        .create_random_region_graph()
        .to_probabilistic_circuit(input_units=2, sum_units=2)
    )
    rows = np.stack(
        [
            (
                np.zeros(10)
                if isinstance(variable, Symbolic)
                else np.random.default_rng(2).normal(size=10)
            )
            for variable in variables
        ],
        axis=1,
    )
    circuit = CirkitCircuit(
        layered=layered, device=torch.device("cpu"), dtype=torch.float64
    )
    np.testing.assert_allclose(
        log_likelihoods_of(circuit=circuit, rows=rows),
        layered.root.log_likelihood_of_nodes(rows)[:, 0],
        rtol=1e-6,
    )
    np.testing.assert_allclose(
        circuit.trained_layered().root.log_likelihood_of_nodes(rows)[:, 0],
        layered.root.log_likelihood_of_nodes(rows)[:, 0],
        rtol=1e-6,
    )


def test_a_sum_over_some_units_of_a_layer_has_no_cirkit_counterpart(
    random_circuit: LayeredProbabilisticCircuit,
):
    """
    A sum unit that mixes only some units of its child layer has no cirkit layer, whose
    sums mix every unit of their inputs.
    """
    input_layer = random_circuit.layers[-1]
    partial = LayerNode(layer=input_layer, node=0).as_circuit(random_circuit.variables)
    with pytest.raises(LayerWithoutCirkitCounterpartError):
        CirkitCircuit(layered=partial, device=torch.device("cpu"), dtype=torch.float64)


# %% benchmark method


def test_cirkit_method_learns_a_circuit_its_cirkit_circuit_agrees_with(
    mixed_dataset: BenchmarkDataset,
):
    """
    The cirkit method trains until it stops and returns a layered circuit whose cirkit
    circuit gives the training rows the same log-likelihoods.
    """
    method = RandomTensorizedSumProductNetworkByCirkit(depth=1, repetitions=2, units=2)
    setting = method.query_settings()[0]
    learned = method.learn(
        dataset=mixed_dataset,
        setting=setting,
        context=TrainingContext(
            convergence=Convergence(maximum_epochs=3), memory_budget_bytes=2**30
        ),
    )
    host = learned.circuit.to_namespace(np)
    rows = host.encoded(mixed_dataset.encoded(frame=mixed_dataset.train))
    circuit = CirkitCircuit(
        layered=host, device=torch.device("cpu"), dtype=torch.float64
    )
    assert learned.history.epochs == 3
    np.testing.assert_allclose(
        log_likelihoods_of(circuit=circuit, rows=rows),
        host.root.log_likelihood_of_nodes(rows)[:, 0],
        rtol=1e-6,
    )


def cirkit_query_rows(
    *,
    method: RandomTensorizedSumProductNetworkByCirkit,
    dataset: BenchmarkDataset,
    setting: ComputationSetting,
) -> list[ExperimentResult]:
    """
    :return: The rows the queries of a random circuit of the method measure in a setting.
    """
    measurement = method.query_measurement(
        dataset=dataset,
        configuration=BenchmarkConfiguration.from_setting(
            learning_method=method.name,
            dataset=dataset.name,
            setting=setting,
            seed=method.seed,
        ),
        setting=setting,
        circuit=method.random_circuit(dataset=dataset),
    )
    measurement.minimum_timing = timedelta(milliseconds=10)
    return measurement.run()


def test_cirkit_queries_in_the_first_query_setting_list_the_exact_queries(
    mixed_dataset: BenchmarkDataset,
):
    """
    The queries of the cirkit method in its first query setting are the likelihood speed
    in cirkit and the queries cirkit answers exactly.
    """
    method = RandomTensorizedSumProductNetworkByCirkit(depth=1, repetitions=2, units=2)
    likelihood, exact = cirkit_query_rows(
        method=method, dataset=mixed_dataset, setting=method.query_settings()[0]
    )
    assert isinstance(likelihood, LikelihoodSpeedResult)
    assert exact.exact_queries == CirkitQueryMeasurement.queries_answered_exactly


def test_cirkit_queries_in_another_setting_measure_only_the_likelihood_speed(
    mixed_dataset: BenchmarkDataset,
):
    """
    In any other query setting, the cirkit method measures only the likelihood speed.
    """
    method = RandomTensorizedSumProductNetworkByCirkit(depth=1, repetitions=2, units=2)
    setting = next(
        setting
        for setting in method.query_settings()[1:]
        if not setting.uses_graphics_processor
    )
    rows = cirkit_query_rows(method=method, dataset=mixed_dataset, setting=setting)
    assert [type(row) for row in rows] == [LikelihoodSpeedResult]

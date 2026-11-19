"""
What the circuit learning benchmark measures of learning a circuit: what it costs and
how well the circuit fits.
"""

from __future__ import annotations

# %% imports
import time
from dataclasses import dataclass
from datetime import timedelta

import jax
import numpy as np
import torch
from krrood_compute_resources.devices import Device
from krrood_compute_resources.memory import HostMemory

from experiments.circuit_learning.datasets import BenchmarkDataset
from experiments.circuit_learning.measurements import ChunkedLogLikelihoodEvaluation
from experiments.circuit_learning.methods import (
    BenchmarkMethod,
    TrainingContext,
)
from experiments.circuit_learning.rows import (
    BenchmarkConfiguration,
    TrainingResult,
)
from experiments.circuit_learning.settings import ComputationSetting
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.base import (
    InnerLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)

# %% training


@dataclass(kw_only=True)
class MeasuredTraining:
    """
    The measurements of a training and the circuit it learned.
    """

    result: TrainingResult
    """
    The measurements.
    """

    circuit: LayeredProbabilisticCircuit
    """
    The learned circuit, its parameters stored as the setting it was learned in says.
    """


@dataclass(kw_only=True)
class TrainingMeasurement:
    """
    Learns a circuit with a method on a dataset in a setting and measures it.
    """

    dataset: BenchmarkDataset
    """
    The dataset.
    """

    method: BenchmarkMethod
    """
    The method.
    """

    setting: ComputationSetting
    """
    The setting to learn in.
    """

    context: TrainingContext
    """
    The limits of training.
    """

    def run(self) -> MeasuredTraining:
        """
        :return: The measurements and the learned circuit.
        """
        host_memory = HostMemory()
        host_memory.reset_peak()
        start = time.perf_counter()
        learned = self.method.learn(
            dataset=self.dataset, setting=self.setting, context=self.context
        )
        training_duration = timedelta(seconds=time.perf_counter() - start)
        peak_memory_bytes = (
            self.peak_device_memory_bytes()
            if self.setting.uses_graphics_processor
            else host_memory.peak_bytes()
        )
        circuit = learned.circuit
        log_likelihoods = ChunkedLogLikelihoodEvaluation(circuit=circuit)
        test = log_likelihoods.compute_log_likelihoods(
            rows=self.dataset.encoded(frame=self.dataset.test)
        )
        train = log_likelihoods.compute_log_likelihoods(
            rows=self.dataset.encoded(frame=self.dataset.train)
        )
        result = TrainingResult(
            configuration=BenchmarkConfiguration.from_setting(
                learning_method=self.method.name,
                dataset=self.dataset.name,
                setting=self.setting,
                seed=self.method.seed,
            ),
            training_duration=training_duration,
            epochs_until_convergence=(
                None if learned.history is None else learned.history.epochs
            ),
            converged=None if learned.history is None else learned.history.converged,
            peak_memory_bytes=peak_memory_bytes,
            number_of_parameters=int(circuit.number_of_parameters),
            number_of_edges=int(
                sum(
                    layer.edges.number_of_edges
                    for layer in circuit.root.all_layers()
                    if isinstance(layer, InnerLayer)
                )
            ),
            average_train_log_likelihood=float(train.mean()),
            average_test_log_likelihood=float(test.mean()),
            fraction_of_test_rows_without_support=float(np.isneginf(test).mean()),
            average_test_log_likelihood_of_supported_rows=self.supported_mean(
                log_likelihoods=test
            ),
        )
        return MeasuredTraining(result=result, circuit=circuit)

    @staticmethod
    def supported_mean(*, log_likelihoods: np.ndarray) -> float:
        """
        :param log_likelihoods: The log-likelihood of every row.
        :return: The mean over the rows of positive likelihood, minus infinity if there
            are none.
        """
        supported = log_likelihoods[~np.isneginf(log_likelihoods)]
        return float(supported.mean()) if len(supported) else -np.inf

    def peak_device_memory_bytes(self) -> int:
        """
        :return: The most GPU memory jax and torch held at once since the process
            started, summed.
        """
        statistics = jax.devices(Device.GRAPHICS_PROCESSOR)[0].memory_stats() or {}
        return int(
            statistics.get("peak_bytes_in_use", 0) + torch.cuda.max_memory_allocated()
        )

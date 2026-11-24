"""
What the circuit learning benchmark measures of a method on a dataset: the cost of
learning, the likelihood of the learned circuit, the speed of its queries and which
queries it answers exactly.
"""

from __future__ import annotations

# %% imports
import resource
import time
from dataclasses import dataclass, field
from enum import StrEnum

import jax
import numpy as np
from random_events.interval import closed
from random_events.product_algebra import SimpleEvent
from random_events.set import Set
from random_events.variable import Symbolic
from sortedcontainers import SortedSet
from typing_extensions import Any, Callable

from probabilistic_model.benchmark.datasets import BenchmarkDataset
from probabilistic_model.benchmark.methods import BenchmarkMethod
from probabilistic_model.probabilistic_circuit.tensorized.inner_layer.sum_layer import (
    SumLayer,
)
from probabilistic_model.probabilistic_circuit.tensorized.layered_probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)


# %% names


class QueryName(StrEnum):
    """
    The queries whose speed is measured.
    """

    LIKELIHOOD = "likelihood"
    CONDITIONING = "conditioning"
    TRUNCATION = "truncation"
    SAMPLING = "sampling"


class Capability(StrEnum):
    """
    The queries whose exactness is checked.
    """

    MARGINAL = "marginal"
    CONDITIONAL = "conditional"
    TRUNCATION = "truncation"
    MOST_LIKELY_ASSIGNMENT = "most_likely_assignment"
    CAUSAL_EFFECT = "causal_effect"


# %% result


@dataclass(kw_only=True)
class BenchmarkResult:
    """
    The measurements of one method on one dataset.
    """

    dataset: str
    """
    The name of the dataset.
    """

    method: str
    """
    The name of the method.
    """

    number_of_variables: int
    """
    The number of variables of the dataset.
    """

    training_seconds: float
    """
    The wall-clock time of learning.
    """

    peak_memory_megabytes: float
    """
    The peak resident memory of the process that learned, in megabytes.
    """

    peak_device_memory_megabytes: float | None
    """
    The peak memory of the accelerator during learning, if there is one.
    """

    number_of_parameters: int
    """
    The number of parameters of the learned circuit.
    """

    train_log_likelihood: float
    """
    The average log-likelihood of the training rows.
    """

    validation_log_likelihood: float
    """
    The average log-likelihood of the validation rows.
    """

    test_log_likelihood: float
    """
    The average log-likelihood of the test rows.
    """

    test_unsupported_fraction: float
    """
    The fraction of the test rows the circuit gives likelihood zero.
    """

    test_log_likelihood_of_supported_rows: float
    """
    The average log-likelihood of the test rows of positive likelihood.
    """

    query_seconds: dict[str, float] = field(default_factory=dict)
    """
    The wall-clock time of every query of :class:`QueryName`.
    """

    capabilities: dict[str, bool] = field(default_factory=dict)
    """
    Whether the circuit answers every query of :class:`Capability` exactly.
    """


# %% measurement


@dataclass(kw_only=True)
class Measurement:
    """
    Learns a circuit with a method on a dataset and measures it.
    """

    dataset: BenchmarkDataset
    """
    The dataset.
    """

    method: BenchmarkMethod
    """
    The method.
    """

    query_rows: int = 1000
    """
    The number of test rows the likelihood is timed on and of samples drawn.
    """

    evaluation_batch_size: int = 4096
    """
    The number of rows evaluated at once when the log-likelihoods are measured.
    """

    def run(self) -> BenchmarkResult:
        """
        :return: The measurements.
        """
        start = time.perf_counter()
        circuit = self.method.learn(dataset=self.dataset)
        training_seconds = time.perf_counter() - start
        host = circuit.to_namespace(np)
        test_log_likelihoods = self.log_likelihoods(
            circuit=host, rows=self.dataset.encoded(frame=self.dataset.test)
        )
        supported = np.isfinite(test_log_likelihoods)
        return BenchmarkResult(
            dataset=self.dataset.name,
            method=str(self.method.name),
            number_of_variables=len(self.dataset.variables),
            training_seconds=training_seconds,
            peak_memory_megabytes=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
            / 1024,
            peak_device_memory_megabytes=self.peak_device_memory_megabytes(),
            number_of_parameters=int(host.number_of_parameters),
            train_log_likelihood=float(
                self.log_likelihoods(
                    circuit=host, rows=self.dataset.encoded(frame=self.dataset.train)
                ).mean()
            ),
            validation_log_likelihood=float(
                self.log_likelihoods(
                    circuit=host,
                    rows=self.dataset.encoded(frame=self.dataset.validation),
                ).mean()
            ),
            test_log_likelihood=float(test_log_likelihoods.mean()),
            test_unsupported_fraction=float(1 - supported.mean()),
            test_log_likelihood_of_supported_rows=float(
                test_log_likelihoods[supported].mean() if supported.any() else -np.inf
            ),
            query_seconds=self.query_seconds(circuit=host),
            capabilities=self.capabilities(circuit=host),
        )

    @staticmethod
    def peak_device_memory_megabytes() -> float | None:
        """
        :return: The peak memory of the first accelerator in megabytes, ``None`` without
            an accelerator that reports it.
        """
        statistics = jax.devices()[0].memory_stats()
        if not statistics or "peak_bytes_in_use" not in statistics:
            return None
        return statistics["peak_bytes_in_use"] / 2**20

    def log_likelihoods(
        self, *, circuit: LayeredProbabilisticCircuit, rows: np.ndarray
    ) -> np.ndarray:
        """
        :param circuit: The circuit with host parameters.
        :param rows: The encoded rows.
        :return: The log-likelihood of every row, evaluated in chunks of
            :attr:`evaluation_batch_size` rows.
        """
        return np.concatenate(
            [
                circuit.log_likelihood(
                    events=rows[start : start + self.evaluation_batch_size]
                )
                for start in range(0, len(rows), self.evaluation_batch_size)
            ]
        )

    def query_seconds(
        self, *, circuit: LayeredProbabilisticCircuit
    ) -> dict[str, float]:
        """
        :param circuit: The circuit with host parameters.
        :return: The time of every query of :class:`QueryName`.
        """
        test = self.dataset.test
        rows = self.dataset.encoded(frame=test)[: self.query_rows]
        variables = list(circuit.variables)
        first_row = test.iloc[0]
        point = {
            variable: first_row[variable.name]
            for variable in variables[: max(len(variables) // 2, 1)]
        }
        queries: dict[QueryName, Callable[[], Any]] = {
            QueryName.LIKELIHOOD: lambda: circuit.log_likelihood(events=rows),
            QueryName.CONDITIONING: lambda: circuit.log_conditional(point),
            QueryName.TRUNCATION: lambda: circuit.log_truncated(
                self.truncation_event(circuit=circuit)
            ),
            QueryName.SAMPLING: lambda: circuit.sample(self.query_rows),
        }
        result = {}
        for name, query in queries.items():
            start = time.perf_counter()
            query()
            result[str(name)] = time.perf_counter() - start
        return result

    def truncation_event(self, *, circuit: LayeredProbabilisticCircuit):
        """
        :param circuit: The circuit.
        :return: The event that restricts the first variable to the value of the first
            test row if it is symbolic, else to the interval [-1, 1].
        """
        variable = circuit.variables[0]
        if isinstance(variable, Symbolic):
            assignment = Set.from_iterable([self.dataset.test.iloc[0][variable.name]])
        else:
            assignment = closed(-1.0, 1.0)
        return SimpleEvent.from_data({variable: assignment}).as_composite_set()

    def capabilities(self, *, circuit: LayeredProbabilisticCircuit) -> dict[str, bool]:
        """
        :param circuit: The circuit with host parameters.
        :return: Whether the circuit answers every query of :class:`Capability` exactly.
        """
        decomposable = bool(circuit.is_decomposable())
        return {
            str(Capability.MARGINAL): decomposable,
            str(Capability.CONDITIONAL): decomposable,
            str(Capability.TRUNCATION): decomposable,
            str(Capability.MOST_LIKELY_ASSIGNMENT): bool(circuit.is_deterministic()),
            str(Capability.CAUSAL_EFFECT): decomposable
            and self.is_support_deterministic_over_first_variable(circuit=circuit),
        }

    @staticmethod
    def is_support_deterministic_over_first_variable(
        *, circuit: LayeredProbabilisticCircuit
    ) -> bool:
        """
        :param circuit: The circuit with host parameters.
        :return: Whether the children of every sum unit whose scope holds the first
            variable have disjoint supports over it, which a backdoor adjustment with
            that variable as the cause needs.
        """
        cause = SortedSet([circuit.variables[0]])
        for layer in circuit.layers:
            if not isinstance(layer, SumLayer) or 0 not in layer.variables:
                continue
            for node in layer.component_supports(circuit.variables):
                supports = [support.marginal(cause) for support in node.supports]
                if len(supports) > 1 and not SumLayer.are_pairwise_disjoint(supports):
                    return False
        return True

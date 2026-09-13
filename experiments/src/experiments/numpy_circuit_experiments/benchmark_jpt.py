from __future__ import annotations
import time
import numpy as np
import pandas as pd
import random
from dataclasses import dataclass
from typing import List, Dict, Any, Tuple
from sklearn.datasets import fetch_california_housing
from probabilistic_model.learning.jpt.jpt import JointProbabilityTree
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import (
    LayeredProbabilisticCircuit,
)
from random_events.variable import Continuous
from random_events.product_algebra import SimpleEvent
from random_events.interval import closed

from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    MeanAndStandardDeviation,
    Unit,
)


from experiments.numpy_circuit_experiments.common import ProbabilisticCircuitBenchmarkResult

@dataclass
class JPTBenchmarkResult(ProbabilisticCircuitBenchmarkResult):
    """
    Results of the JPT-based benchmark.
    """


def run_jpt_benchmark(iterations: int = 3) -> ExperimentsTable:
    """
    Run the JPT benchmark and return the results as a table.
    """
    print("Loading California Housing dataset...")
    data = fetch_california_housing(as_frame=True)
    dataframe = data.frame

    print("Training JPT (this might take a minute)...")
    variables = infer_variables_from_dataframe(dataframe)
    joint_probability_tree = JointProbabilityTree(
        annotated_variables=variables, min_samples_per_leaf=100, max_depth=10
    )
    rustworkx_probabilistic_circuit = joint_probability_tree.fit(dataframe)
    layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

    batch_sizes = [1000, 10000, 20000]
    results = []

    for batch_size in batch_sizes:
        test_data = dataframe.sample(n=batch_size, replace=True).values
        results.extend(
            run_inference_benchmarks(
                rustworkx_probabilistic_circuit,
                layered_probabilistic_circuit,
                test_data,
                batch_size,
                iterations,
            )
        )

    results.extend(
        run_truncation_benchmarks(
            rustworkx_probabilistic_circuit,
            layered_probabilistic_circuit,
            dataframe,
            iterations,
        )
    )

    return ExperimentsTable(results)


def run_inference_benchmarks(
    rustworkx_probabilistic_circuit,
    layered_probabilistic_circuit,
    test_data,
    batch_size,
    iterations,
) -> List[JPTBenchmarkResult]:
    results = []
    # Likelihood
    rustworkx_times = []
    layered_times = []
    for _ in range(iterations):
        start_time = time.time()
        rustworkx_probabilistic_circuit.log_likelihood(test_data)
        rustworkx_times.append(time.time() - start_time)
        start_time = time.time()
        layered_probabilistic_circuit.log_likelihood(test_data)
        layered_times.append(time.time() - start_time)

    rustworkx_measurements = MeanAndStandardDeviation.from_measurements(rustworkx_times, Unit.SECONDS)
    layered_measurements = MeanAndStandardDeviation.from_measurements(layered_times, Unit.SECONDS)
    results.append(
        JPTBenchmarkResult(
            str(batch_size),
            "Likelihood",
            rustworkx_measurements,
            layered_measurements,
            rustworkx_measurements.mean / layered_measurements.mean if layered_measurements.mean > 0 else float("inf"),
        )
    )

    # Sampling
    rustworkx_times = []
    layered_times = []
    for _ in range(iterations):
        start_time = time.time()
        rustworkx_probabilistic_circuit.sample(batch_size)
        rustworkx_times.append(time.time() - start_time)
        start_time = time.time()
        layered_probabilistic_circuit.sample(batch_size)
        layered_times.append(time.time() - start_time)

    rustworkx_measurements = MeanAndStandardDeviation.from_measurements(rustworkx_times, Unit.SECONDS)
    layered_measurements = MeanAndStandardDeviation.from_measurements(layered_times, Unit.SECONDS)
    results.append(
        JPTBenchmarkResult(
            str(batch_size),
            "Sampling",
            rustworkx_measurements,
            layered_measurements,
            rustworkx_measurements.mean / layered_measurements.mean if layered_measurements.mean > 0 else float("inf"),
        )
    )
    return results


def run_truncation_benchmarks(
    rustworkx_probabilistic_circuit,
    layered_probabilistic_circuit,
    dataframe,
    iterations,
) -> List[JPTBenchmarkResult]:
    results = []
    # Truncation
    truncation_configurations = [1, 10, 50]
    for number_of_simple_sets in truncation_configurations:
        events = []
        for _ in range(2):
            composite_event = None
            for _ in range(number_of_simple_sets):
                event_data = {}
                for variable in rustworkx_probabilistic_circuit.variables:
                    if isinstance(variable, Continuous):
                        variable_min = dataframe[variable.name].min()
                        variable_max = dataframe[variable.name].max()
                        lower, upper = sorted(
                            [random.uniform(variable_min, variable_max), random.uniform(variable_min, variable_max)]
                        )
                        event_data[variable] = closed(lower, upper)
                simple_event = SimpleEvent.from_data(event_data).as_composite_set()
                if composite_event is None:
                    composite_event = simple_event
                else:
                    composite_event = composite_event | simple_event
            events.append(composite_event)

        rustworkx_times = []
        layered_times = []
        for _ in range(iterations):
            total_rustworkx_time = 0
            total_layered_time = 0
            for event in events:
                start_time = time.time()
                rustworkx_probabilistic_circuit.log_truncated(event)
                total_rustworkx_time += time.time() - start_time
                start_time = time.time()
                layered_probabilistic_circuit.log_truncated(event)
                total_layered_time += time.time() - start_time
            rustworkx_times.append(total_rustworkx_time / len(events))
            layered_times.append(total_layered_time / len(events))

        rustworkx_measurements = MeanAndStandardDeviation.from_measurements(rustworkx_times, Unit.SECONDS)
        layered_measurements = MeanAndStandardDeviation.from_measurements(layered_times, Unit.SECONDS)
        results.append(
            JPTBenchmarkResult(
                f"{number_of_simple_sets} sets",
                "Truncation",
                rustworkx_measurements,
                layered_measurements,
                rustworkx_measurements.mean / layered_measurements.mean if layered_measurements.mean > 0 else float("inf"),
            )
        )
    return results


if __name__ == "__main__":
    table = run_jpt_benchmark()
    print(table.render())

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
    ProbabilisticCircuit as NumPyProbabilisticCircuit,
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


@dataclass
class JPTBenchmarkResult(ExperimentResult):
    """
    Results of the JPT-based benchmark.
    """

    batch_size: str
    """
    The batch size (or number of sets for truncation).
    """

    operation: str
    """
    The inference operation performed.
    """

    rx_duration: MeanAndStandardDeviation
    """
    Duration of the Rustworkx implementation.
    """

    np_duration: MeanAndStandardDeviation
    """
    Duration of the NumPy implementation.
    """

    speedup: float
    """
    Calculated speedup (rx / np).
    """


def run_jpt_benchmark(iterations: int = 3) -> ExperimentsTable:
    """
    Run the JPT benchmark and return the results as a table.
    """
    print("Loading California Housing dataset...")
    data = fetch_california_housing(as_frame=True)
    df = data.frame

    print("Training JPT (this might take a minute)...")
    variables = infer_variables_from_dataframe(df)
    jpt = JointProbabilityTree(
        annotated_variables=variables, min_samples_per_leaf=100, max_depth=10
    )
    rx_pc = jpt.fit(df)
    np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

    batch_sizes = [1000, 10000, 20000]
    results = []

    for batch_size in batch_sizes:
        test_data = df.sample(n=batch_size, replace=True).values

        # Likelihood
        rx_times = []
        np_times = []
        for _ in range(iterations):
            t0 = time.time()
            rx_pc.log_likelihood(test_data)
            rx_times.append(time.time() - t0)
            t0 = time.time()
            np_pc.log_likelihood(test_data)
            np_times.append(time.time() - t0)

        rx_ms = MeanAndStandardDeviation.from_measurements(rx_times, Unit.SECONDS)
        np_ms = MeanAndStandardDeviation.from_measurements(np_times, Unit.SECONDS)
        results.append(
            JPTBenchmarkResult(
                str(batch_size),
                "Likelihood",
                rx_ms,
                np_ms,
                rx_ms.mean / np_ms.mean if np_ms.mean > 0 else float("inf"),
            )
        )

        # Sampling
        rx_times = []
        np_times = []
        for _ in range(iterations):
            t0 = time.time()
            rx_pc.sample(batch_size)
            rx_times.append(time.time() - t0)
            t0 = time.time()
            np_pc.sample(batch_size)
            np_times.append(time.time() - t0)

        rx_ms = MeanAndStandardDeviation.from_measurements(rx_times, Unit.SECONDS)
        np_ms = MeanAndStandardDeviation.from_measurements(np_times, Unit.SECONDS)
        results.append(
            JPTBenchmarkResult(
                str(batch_size),
                "Sampling",
                rx_ms,
                np_ms,
                rx_ms.mean / np_ms.mean if np_ms.mean > 0 else float("inf"),
            )
        )

    # Truncation
    trunc_configs = [1, 10, 50]
    for num_simple_sets in trunc_configs:
        events = []
        for _ in range(2):
            composite_event = None
            for _ in range(num_simple_sets):
                event_data = {}
                for var in rx_pc.variables:
                    if isinstance(var, Continuous):
                        v_min = df[var.name].min()
                        v_max = df[var.name].max()
                        l, r = sorted(
                            [random.uniform(v_min, v_max), random.uniform(v_min, v_max)]
                        )
                        event_data[var] = closed(l, r)
                simple_event = SimpleEvent.from_data(event_data).as_composite_set()
                if composite_event is None:
                    composite_event = simple_event
                else:
                    composite_event = composite_event | simple_event
            events.append(composite_event)

        rx_times = []
        np_times = []
        for _ in range(iterations):
            t_rx = 0
            t_np = 0
            for event in events:
                t0 = time.time()
                rx_pc.log_truncated(event)
                t_rx += time.time() - t0
                t0 = time.time()
                np_pc.log_truncated(event)
                t_np += time.time() - t0
            rx_times.append(t_rx / len(events))
            np_times.append(t_np / len(events))

        rx_ms = MeanAndStandardDeviation.from_measurements(rx_times, Unit.SECONDS)
        np_ms = MeanAndStandardDeviation.from_measurements(np_times, Unit.SECONDS)
        results.append(
            JPTBenchmarkResult(
                f"{num_simple_sets} sets",
                "Truncation",
                rx_ms,
                np_ms,
                rx_ms.mean / np_ms.mean if np_ms.mean > 0 else float("inf"),
            )
        )

    return ExperimentsTable(results)


if __name__ == "__main__":
    table = run_jpt_benchmark()
    print(table.render())

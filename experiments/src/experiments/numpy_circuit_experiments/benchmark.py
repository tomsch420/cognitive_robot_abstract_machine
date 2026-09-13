from __future__ import annotations
import time
import numpy as np
import random
from dataclasses import dataclass
from typing_extensions import List, Dict, Any, Tuple
from sortedcontainers import SortedSet

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit as RXProbabilisticCircuit,
    SumUnit,
    ProductUnit,
    leaf,
)
from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import (
    ProbabilisticCircuit as NumPyProbabilisticCircuit,
)
from probabilistic_model.distributions.gaussian import GaussianDistribution
from probabilistic_model.distributions.distributions import SymbolicDistribution
from random_events.variable import Continuous, Symbolic
from random_events.product_algebra import SimpleEvent
from random_events.interval import closed
from random_events.set import Set

from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    MeanAndStandardDeviation,
    Unit,
)


@dataclass
class NumPyCircuitBenchmarkResult(ExperimentResult):
    """
    Results of a single benchmark run comparing Rustworkx and NumPy implementations.
    """

    depth: int
    """
    The depth of the random circuit.
    """

    width: int
    """
    The width of each layer in the random circuit.
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


def generate_random_circuit(variables, depth, width):
    pc = RXProbabilisticCircuit()

    current_layer = []
    for var in variables:
        for _ in range(width):
            if isinstance(var, Continuous):
                dist = GaussianDistribution(
                    variable=var,
                    location=random.uniform(-5, 5),
                    scale=random.uniform(0.5, 2.0),
                )
            else:
                domain_elements = list(var.domain.simple_sets)
                probs = np.random.dirichlet(np.ones(len(domain_elements)))
                dist = SymbolicDistribution(
                    variable=var,
                    probabilities={
                        hash(val): p for val, p in zip(domain_elements, probs)
                    },
                )
            l = leaf(dist, pc)
            current_layer.append(l)

    for d in range(depth):
        product_layer = []
        if len(variables) > 1:
            nodes_by_var = {
                var: [n for n in current_layer if var in n.variables]
                for var in variables
            }
            unused_nodes = set(current_layer)

            while unused_nodes or len(product_layer) < width:
                p = ProductUnit(probabilistic_circuit=pc)
                for var in variables:
                    var_unused = [n for n in unused_nodes if var in n.variables]
                    if var_unused:
                        node = random.choice(var_unused)
                        unused_nodes.remove(node)
                    else:
                        node = random.choice(nodes_by_var[var])
                    p.add_subcircuit(node)
                product_layer.append(p)
        else:
            product_layer = current_layer

        sum_layer = []
        unused_nodes = set(product_layer)
        while unused_nodes or len(sum_layer) < width:
            s = SumUnit(probabilistic_circuit=pc)
            if unused_nodes:
                node = random.choice(list(unused_nodes))
                unused_nodes.remove(node)
                children = [node]
            else:
                children = [random.choice(product_layer)]

            num_extra = random.randint(1, 4)
            if len(product_layer) > 1:
                extra_children = random.sample(
                    product_layer, min(len(product_layer), num_extra)
                )
                for c in extra_children:
                    if c not in children:
                        children.append(c)

            weights = np.random.dirichlet(np.ones(len(children)))
            for child, weight in zip(children, weights):
                s.add_subcircuit(child, np.log(weight))
            sum_layer.append(s)
        current_layer = sum_layer

    root = SumUnit(probabilistic_circuit=pc)
    weights = np.random.dirichlet(np.ones(len(current_layer)))
    for child, weight in zip(current_layer, weights):
        root.add_subcircuit(child, np.log(weight))

    return pc


def run_random_circuit_benchmark(iterations: int = 3) -> ExperimentsTable:
    """
    Run the random circuit benchmark and return the results as a table.
    """
    np.random.seed(42)
    random.seed(42)

    c_vars = [Continuous(f"c{i}") for i in range(2)]
    s_vars = [
        Symbolic(f"s{i}", domain=Set.from_iterable(["a", "b", "c"])) for i in range(1)
    ]
    variables = c_vars + s_vars

    configs = [(1, 5), (2, 10), (3, 20), (4, 40)]
    batch_sizes = [1000, 10000]

    results = []

    for depth, width in configs:
        rx_pc = generate_random_circuit(variables, depth, width)
        np_pc = NumPyProbabilisticCircuit.from_rustworkx(rx_pc)

        for batch_size in batch_sizes:
            # Construct data
            data = np.empty((batch_size, len(variables)), dtype=object)
            for i, var in enumerate(variables):
                if isinstance(var, Continuous):
                    data[:, i] = np.random.randn(batch_size)
                else:
                    domain_elements = list(var.domain.simple_sets)
                    data[:, i] = np.random.choice(domain_elements, size=batch_size)

            # Likelihood
            rx_times = []
            np_times = []
            for _ in range(iterations):
                t0 = time.time()
                rx_pc.log_likelihood(data)
                rx_times.append(time.time() - t0)
                t0 = time.time()
                np_pc.log_likelihood(data)
                np_times.append(time.time() - t0)

            rx_ms = MeanAndStandardDeviation.from_measurements(rx_times, Unit.SECONDS)
            np_ms = MeanAndStandardDeviation.from_measurements(np_times, Unit.SECONDS)
            results.append(
                NumPyCircuitBenchmarkResult(
                    depth,
                    width,
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
                NumPyCircuitBenchmarkResult(
                    depth,
                    width,
                    str(batch_size),
                    "Sampling",
                    rx_ms,
                    np_ms,
                    rx_ms.mean / np_ms.mean if np_ms.mean > 0 else float("inf"),
                )
            )

        # Truncation
        trunc_configs = [1, 10, 100]
        for num_simple_sets in trunc_configs:
            events = []
            for _ in range(3):
                composite_event = None
                for _ in range(num_simple_sets):
                    event_data = {}
                    for v in variables:
                        if isinstance(v, Continuous):
                            l, r = sorted([random.uniform(-2, 2), random.uniform(-2, 2)])
                            event_data[v] = closed(l, r)
                        else:
                            domain_elements = [se.element for se in v.domain.simple_sets]
                            event_data[v] = v.make_value(
                                random.sample(
                                    domain_elements, random.randint(1, len(domain_elements))
                                )
                            )
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
                NumPyCircuitBenchmarkResult(
                    depth,
                    width,
                    f"{num_simple_sets} sets",
                    "Truncation",
                    rx_ms,
                    np_ms,
                    rx_ms.mean / np_ms.mean if np_ms.mean > 0 else float("inf"),
                )
            )

    return ExperimentsTable(results)


if __name__ == "__main__":
    table = run_random_circuit_benchmark()
    print(table.render())

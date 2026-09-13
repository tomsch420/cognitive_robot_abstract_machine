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
    LayeredProbabilisticCircuit,
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


from experiments.numpy_circuit_experiments.common import (
    ProbabilisticCircuitBenchmarkResult,
    generate_random_events,
)

@dataclass
class NumPyCircuitBenchmarkResult(ProbabilisticCircuitBenchmarkResult):
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


def _create_leaf_layer(variables, width, probabilistic_circuit):
    leaf_nodes = []
    for variable in variables:
        for _ in range(width):
            if isinstance(variable, Continuous):
                distribution = GaussianDistribution(
                    variable=variable,
                    location=random.uniform(-5, 5),
                    scale=random.uniform(0.5, 2.0),
                )
            else:
                domain_elements = list(variable.domain.simple_sets)
                probabilities = np.random.dirichlet(np.ones(len(domain_elements)))
                distribution = SymbolicDistribution(
                    variable=variable,
                    probabilities={
                        hash(value): probability
                        for value, probability in zip(domain_elements, probabilities)
                    },
                )
            leaf_nodes.append(leaf(distribution, probabilistic_circuit))
    return leaf_nodes


def _create_product_layer(variables, current_layer, width, probabilistic_circuit):
    if len(variables) == 1:
        return current_layer

    product_layer = []
    nodes_by_variable = {
        variable: [node for node in current_layer if variable in node.variables]
        for variable in variables
    }
    unused_nodes = set(current_layer)

    while unused_nodes or len(product_layer) < width:
        product_unit = ProductUnit(probabilistic_circuit=probabilistic_circuit)
        for variable in variables:
            variable_unused = [
                node for node in unused_nodes if variable in node.variables
            ]
            if variable_unused:
                node = random.choice(variable_unused)
                unused_nodes.remove(node)
            else:
                node = random.choice(nodes_by_variable[variable])
            product_unit.add_subcircuit(node)
        product_layer.append(product_unit)
    return product_layer


def _create_sum_layer(product_layer, width, probabilistic_circuit):
    sum_layer = []
    unused_nodes = set(product_layer)
    while unused_nodes or len(sum_layer) < width:
        sum_unit = SumUnit(probabilistic_circuit=probabilistic_circuit)
        if unused_nodes:
            node = random.choice(list(unused_nodes))
            unused_nodes.remove(node)
            children = [node]
        else:
            children = [random.choice(product_layer)]

        number_of_extra_children = random.randint(1, 4)
        if len(product_layer) > 1:
            extra_children = random.sample(
                product_layer, min(len(product_layer), number_of_extra_children)
            )
            for child in extra_children:
                if child not in children:
                    children.append(child)

        weights = np.random.dirichlet(np.ones(len(children)))
        for child, weight in zip(children, weights):
            sum_unit.add_subcircuit(child, np.log(weight))
        sum_layer.append(sum_unit)
    return sum_layer


def generate_random_circuit(variables, depth, width):
    probabilistic_circuit = RXProbabilisticCircuit()
    current_layer = _create_leaf_layer(variables, width, probabilistic_circuit)

    for _ in range(depth):
        product_layer = _create_product_layer(
            variables, current_layer, width, probabilistic_circuit
        )
        current_layer = _create_sum_layer(product_layer, width, probabilistic_circuit)

    root = SumUnit(probabilistic_circuit=probabilistic_circuit)
    weights = np.random.dirichlet(np.ones(len(current_layer)))
    for child, weight in zip(current_layer, weights):
        root.add_subcircuit(child, np.log(weight))

    return probabilistic_circuit


def run_random_circuit_benchmark(iterations: int = 3) -> ExperimentsTable:
    """
    Run the random circuit benchmark and return the results as a table.
    """
    np.random.seed(42)
    random.seed(42)

    continuous_variables = [Continuous(f"c{i}") for i in range(2)]
    symbolic_variables = [
        Symbolic(f"s{i}", domain=Set.from_iterable(["a", "b", "c"])) for i in range(1)
    ]
    variables = continuous_variables + symbolic_variables

    configurations = [(1, 5), (2, 10), (3, 20), (4, 40)]
    batch_sizes = [1000, 10000]

    results = []

    for depth, width in configurations:
        rustworkx_probabilistic_circuit = generate_random_circuit(variables, depth, width)
        layered_probabilistic_circuit = LayeredProbabilisticCircuit.from_rustworkx(rustworkx_probabilistic_circuit)

        for batch_size in batch_sizes:
            results.extend(
                run_inference_benchmarks(
                    rustworkx_probabilistic_circuit,
                    layered_probabilistic_circuit,
                    variables,
                    batch_size,
                    depth,
                    width,
                    iterations,
                )
            )

        results.extend(
            run_truncation_benchmarks(
                rustworkx_probabilistic_circuit,
                layered_probabilistic_circuit,
                variables,
                depth,
                width,
                iterations,
            )
        )

    return ExperimentsTable(results)


def run_inference_benchmarks(
    rustworkx_probabilistic_circuit,
    layered_probabilistic_circuit,
    variables,
    batch_size,
    depth,
    width,
    iterations,
) -> List[NumPyCircuitBenchmarkResult]:
    # Construct data
    data = np.empty((batch_size, len(variables)), dtype=object)
    for i, variable in enumerate(variables):
        if isinstance(variable, Continuous):
            data[:, i] = np.random.randn(batch_size)
        else:
            domain_elements = list(variable.domain.simple_sets)
            data[:, i] = np.random.choice(domain_elements, size=batch_size)

    results = []
    # Likelihood
    rustworkx_times = []
    layered_times = []
    for _ in range(iterations):
        start_time = time.time()
        rustworkx_probabilistic_circuit.log_likelihood(data)
        rustworkx_times.append(time.time() - start_time)
        start_time = time.time()
        layered_probabilistic_circuit.log_likelihood(data)
        layered_times.append(time.time() - start_time)

    rustworkx_measurements = MeanAndStandardDeviation.from_measurements(rustworkx_times, Unit.SECONDS)
    layered_measurements = MeanAndStandardDeviation.from_measurements(layered_times, Unit.SECONDS)
    results.append(
        NumPyCircuitBenchmarkResult(
            str(batch_size),
            "Likelihood",
            rustworkx_measurements,
            layered_measurements,
            rustworkx_measurements.mean / layered_measurements.mean if layered_measurements.mean > 0 else float("inf"),
            depth,
            width,
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
        NumPyCircuitBenchmarkResult(
            str(batch_size),
            "Sampling",
            rustworkx_measurements,
            layered_measurements,
            rustworkx_measurements.mean / layered_measurements.mean if layered_measurements.mean > 0 else float("inf"),
            depth,
            width,
        )
    )
    return results


def run_truncation_benchmarks(
    rustworkx_probabilistic_circuit,
    layered_probabilistic_circuit,
    variables,
    depth,
    width,
    iterations,
) -> List[NumPyCircuitBenchmarkResult]:
    results = []
    truncation_configurations = [1, 10, 100]
    for number_of_simple_sets in truncation_configurations:
        events = generate_random_events(variables, number_of_simple_sets)

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

        rustworkx_measurements = MeanAndStandardDeviation.from_measurements(
            rustworkx_times, Unit.SECONDS
        )
        layered_measurements = MeanAndStandardDeviation.from_measurements(
            layered_times, Unit.SECONDS
        )
        results.append(
            NumPyCircuitBenchmarkResult(
                f"{number_of_simple_sets} sets",
                "Truncation",
                rustworkx_measurements,
                layered_measurements,
                rustworkx_measurements.mean / layered_measurements.mean
                if layered_measurements.mean > 0
                else float("inf"),
                depth,
                width,
            )
        )
    return results


if __name__ == "__main__":
    table = run_random_circuit_benchmark()
    print(table.render())

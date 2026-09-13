from __future__ import annotations
from dataclasses import dataclass
import random
from typing import List, Iterable
from random_events.variable import Continuous, Variable
from random_events.product_algebra import SimpleEvent
from random_events.interval import closed
from experiments.experiment_definitions import ExperimentResult, MeanAndStandardDeviation

@dataclass
class ProbabilisticCircuitBenchmarkResult(ExperimentResult):
    """
    Common result structure for probabilistic circuit benchmarks.
    """
    batch_size: str
    """
    The batch size (or number of sets for truncation).
    """
    operation: str
    """
    The inference operation performed.
    """
    rustworkx_duration: MeanAndStandardDeviation
    """
    Duration of the Rustworkx implementation.
    """
    numpy_duration: MeanAndStandardDeviation
    """
    Duration of the NumPy implementation.
    """
    speedup: float
    """
    Calculated speedup (rustworkx / numpy).
    """


def generate_random_events(
    variables: Iterable[Variable], number_of_simple_sets: int, number_of_events: int = 3
):
    """
    Generate random events for truncation benchmarks.
    """
    events = []
    for _ in range(number_of_events):
        composite_event = None
        for _ in range(number_of_simple_sets):
            event_data = {}
            for variable in variables:
                if isinstance(variable, Continuous):
                    lower, upper = sorted([random.uniform(-2, 2), random.uniform(-2, 2)])
                    event_data[variable] = closed(lower, upper)
                else:
                    domain_elements = [
                        simple_set.element for simple_set in variable.domain.simple_sets
                    ]
                    event_data[variable] = variable.make_value(
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
    return events

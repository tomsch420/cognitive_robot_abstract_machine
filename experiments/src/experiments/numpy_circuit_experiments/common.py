from __future__ import annotations
from dataclasses import dataclass
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

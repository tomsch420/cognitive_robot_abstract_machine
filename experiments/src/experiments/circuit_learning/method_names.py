"""
The names of the learning methods of the circuit learning benchmark.
"""

from __future__ import annotations

# %% imports
from enum import StrEnum

# %% names


class LearningMethod(StrEnum):
    """
    A learning method of the benchmark, baselines included.
    """

    RANDOM_TENSORIZED_SUM_PRODUCT_NETWORK_GRADIENT_DESCENT = "RAT-SPN, gradient descent"
    RANDOM_TENSORIZED_SUM_PRODUCT_NETWORK_EXPECTATION_MAXIMIZATION = (
        "RAT-SPN, expectation maximization"
    )
    RANDOM_TENSORIZED_SUM_PRODUCT_NETWORK_CIRKIT = "RAT-SPN, cirkit"
    JOINT_PROBABILITY_TREE = "joint probability tree"

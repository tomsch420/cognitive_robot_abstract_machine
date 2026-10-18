"""
Telling from the rounds of a policy iteration whether the policy still changes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass, field

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Any, Dict, List, Optional

from experiments.grasp_learning.convergence import (
    ConvergenceMeter,
    SamplingDensities,
    SamplingPhase,
)
from experiments.grasp_learning.grasp_variables import grasp_features
from experiments.grasp_learning.models import GraspModel
from experiments.grasp_learning.records import GraspAttempt
from semantic_digital_twin.grasping.surface_grasp import draw_surface_grasps

# %% telling when to stop


@dataclass
class RoundState:
    """
    What a stopping rule may look at in one round.
    """

    number: int
    """
    The round, from 1.
    """

    model: GraspModel
    """
    The round's model, learned from :attr:`attempts`.
    """

    attempts: Dict[int, GraspAttempt]
    """
    Every attempt the model was learned from, by the number it was stored under.
    """

    phases: List[SamplingPhase]
    """
    The phases the attempts were drawn in.
    """

    densities: SamplingDensities
    """
    The distributions the attempts were drawn from.
    """

    main_object: str
    """
    The object whose part the rule asks the model about.
    """


class StoppingRule(ABC):
    """
    Tells from each round whether the policy no longer changes.
    """

    @abstractmethod
    def observe(self, state: RoundState) -> Dict[str, Any]:
        """
        Look at a round.

        :param state: The round.
        :return: What the rule measured, for the round's report.
        """

    @property
    @abstractmethod
    def stops(self) -> bool:
        """
        Whether the rounds observed so far say the policy no longer changes.
        """


@dataclass
class FixedRounds(StoppingRule):
    """
    Never stops early: the iteration runs for its maximum number of rounds.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        return {}

    @property
    def stops(self) -> bool:
        return False


@dataclass
class PolicyUnchanged(StoppingRule):
    """
    Stops once grasps drawn from consecutive models cannot be told apart for some rounds
    in a row, by a permutation test of their maximum mean discrepancy.
    """

    policy_samples: int = 400
    """
    How many grasps are drawn from each model to compare it with the previous one.
    """

    significance: float = 0.05
    """
    Below this p-value two consecutive models count as different.
    """

    required_stable_rounds: int = 2
    """
    How many rounds in a row the model has to stay the same.
    """

    number_of_permutations: int = 500
    """
    How many permutations the test draws.
    """

    seed: int = 0
    """
    Seeds the permutations.
    """

    stable_rounds: int = field(default=0, init=False)
    """
    How many rounds in a row the model stayed the same so far.
    """

    previous_samples: Optional[NDArray[np.float64]] = field(default=None, init=False)
    """
    The grasps drawn from the previous round's model.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        part = state.densities.parts[state.main_object]
        grasps = draw_surface_grasps(
            part.surface_grasp_statement(require_lifting=True),
            self.policy_samples,
            state.model.model_registry(),
        )
        samples = np.array([grasp_features(grasp) for grasp in grasps])
        p_value = None
        if (
            self.previous_samples is not None
            and len(samples)
            and len(self.previous_samples)
        ):
            p_value = self.p_value(
                self.previous_samples,
                samples,
                np.random.default_rng((self.seed, state.number)),
            )
            self.stable_rounds = (
                self.stable_rounds + 1 if p_value >= self.significance else 0
            )
        self.previous_samples = samples
        return {"policy_change_p_value": p_value, "stable_rounds": self.stable_rounds}

    @property
    def stops(self) -> bool:
        return self.stable_rounds >= self.required_stable_rounds

    def p_value(
        self,
        previous: NDArray[np.float64],
        current: NDArray[np.float64],
        generator: np.random.Generator,
    ) -> float:
        """
        :param previous: Grasps drawn from one policy, one row per grasp.
        :param current: Grasps drawn from another policy.
        :param generator: Draws the permutations.
        :return: The share of permutations whose maximum mean discrepancy under a
            Gaussian kernel is at least the observed one; small when the policies
            differ.
        """
        pooled = np.vstack([previous, current])
        pooled = (pooled - pooled.mean(axis=0)) / np.maximum(pooled.std(axis=0), 1e-9)
        squared_distances = np.sum(
            (pooled[:, None, :] - pooled[None, :, :]) ** 2, axis=-1
        )
        bandwidth = np.median(squared_distances[squared_distances > 0])
        kernel = np.exp(-squared_distances / bandwidth)
        labels = np.r_[np.zeros(len(previous), bool), np.ones(len(current), bool)]

        def discrepancy(in_current: NDArray[np.bool_]) -> float:
            in_previous = ~in_current
            return (
                kernel[np.ix_(in_previous, in_previous)].mean()
                + kernel[np.ix_(in_current, in_current)].mean()
                - 2 * kernel[np.ix_(in_previous, in_current)].mean()
            )

        observed = discrepancy(labels)
        exceeding = sum(
            discrepancy(generator.permutation(labels)) >= observed
            for _ in range(self.number_of_permutations)
        )
        return (exceeding + 1) / (self.number_of_permutations + 1)


@dataclass
class DistancesWithinFloors(StoppingRule):
    """
    Stops once both L1 distances of :mod:`experiments.grasp_learning.convergence` stay
    within their floors for some rounds in a row.
    """

    required_converged_rounds: int = 3
    """
    How many rounds in a row both distances have to stay within their floors.
    """

    sample_size: int = 5000
    """
    How many samples each distance is estimated from.
    """

    seed: int = 0
    """
    Seeds the resampling.
    """

    meter: Optional[ConvergenceMeter] = field(default=None, init=False)
    """
    Measures the distances; made in the first round.
    """

    converged_rounds: int = field(default=0, init=False)
    """
    How many rounds in a row both distances stayed within their floors so far.
    """

    def observe(self, state: RoundState) -> Dict[str, Any]:
        if self.meter is None:
            self.meter = ConvergenceMeter(
                task=state.model.task,
                main_object=state.main_object,
                densities=state.densities,
                sample_size=self.sample_size,
                seed=self.seed,
            )
        measurements = self.meter.measure(state.number, state.attempts, state.phases)
        self.converged_rounds = (
            self.converged_rounds + 1 if measurements.converged else 0
        )
        return {
            "measurements": asdict(measurements),
            "converged_rounds": self.converged_rounds,
        }

    @property
    def stops(self) -> bool:
        return self.converged_rounds >= self.required_converged_rounds

"""
Measuring whether learned grasp models still change, on the whole distribution over the
grasp and its result.

Attempts are drawn from changing distributions: uniformly from what an object's
annotation states, and from each round's learned policy. A model learned from all of
them describes the mix of those distributions as much as the world. Two measures tell
the world apart from the mix:

- **conditional L1**: the L1 distance between the distributions of results given the
  grasp, averaged over grasps drawn uniformly from what the statement allows. Only the
  physics determines these, whatever drew the grasps.
- **weighted joint L1**: the L1 distance between joint models learned from the attempts
  reweighted to grasps drawn uniformly, by importance weights against the mixture of
  every distribution that drew grasps for the object.

Each is compared with a floor: the same measure between two models learned from two
resamples of the same attempts, which is how much learning alone changes a model.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Dict, List, Optional, Tuple

from experiments.grasp_learning.grasp_variables import SurfaceGraspVariables
from experiments.grasp_learning.models import GraspModel, GraspModelLearner
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from krrood.parametrization.model_registries import UniformPriorRegistry
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% where the attempts came from


@dataclass
class SamplingPhase:
    """
    Consecutive attempts on one object whose grasps were drawn from one distribution.
    """

    object_name: str
    """
    The name of the object's body.
    """

    source: GraspSource
    """
    Whether the grasps were drawn uniformly or from a model.
    """

    model_index: Optional[int]
    """
    The position of the model among the iteration's models, for grasps drawn from one;
    ``None`` for uniform grasps.
    """

    first_number: int
    """
    The number the first attempt of the phase was stored under.
    """

    last_number: int
    """
    The number the last attempt of the phase was stored under.
    """

    @classmethod
    def read_all(cls, path: Path) -> List[SamplingPhase]:
        """
        :param path: A JSON lines file of phases.
        :return: The phases, in order; none if the file does not exist.
        """
        if not path.exists():
            return []
        phases = []
        for line in path.read_text().splitlines():
            values = json.loads(line)
            values["source"] = GraspSource(values["source"])
            phases.append(cls(**values))
        return phases

    def append_to(self, path: Path) -> None:
        """
        :param path: The JSON lines file to append this phase to.
        """
        values = asdict(self)
        values["source"] = self.source.value
        with path.open("a") as phases:
            phases.write(json.dumps(values) + "\n")

    def holds(self, number: int, object_name: str) -> bool:
        """
        :param number: The number an attempt was stored under.
        :param object_name: The object it was tried on.
        :return: Whether the attempt was drawn in this phase.
        """
        return (
            object_name == self.object_name
            and self.first_number <= number <= self.last_number
        )


# %% densities of the distributions grasps were drawn from


@dataclass
class SamplingDensities:
    """
    The densities over grasps of the distributions attempts were drawn from.
    """

    parts: Dict[str, HasGraspCandidates]
    """
    The grasped part of each object, by the name of the object's body.
    """

    models: List[GraspModel]
    """
    The models grasps were drawn from, by their index.
    """

    circuits: Dict[Tuple[str, Optional[int]], ProbabilisticCircuit] = field(
        default_factory=dict, init=False, repr=False
    )
    """
    The distributions worked out so far, by object and model index.
    """

    def of(self, object_name: str, model_index: Optional[int]) -> ProbabilisticCircuit:
        """
        :param object_name: The object the grasps were drawn for.
        :param model_index: The model they were drawn from; ``None`` for uniformly.
        :return: The distribution the grasps were drawn from, over the grasp alone.
        """
        key = (object_name, model_index)
        if key in self.circuits:
            return self.circuits[key]
        part = self.parts[object_name]
        if model_index is None:
            parameters = UnderspecifiedParameters(part.surface_grasp_statement())
            model = UniformPriorRegistry().get_model(parameters)
        else:
            parameters = UnderspecifiedParameters(
                part.surface_grasp_statement(require_lifting=True)
            )
            model = self.models[model_index].model_registry().get_model(parameters)
        truncated = parameters.resolve_conditioned_and_truncated_model(model)
        self.circuits[key] = truncated.marginal(
            SurfaceGraspVariables(truncated).of_the_grasp
        )
        return self.circuits[key]

    def likelihood(
        self, object_name: str, model_index: Optional[int], attempts: List[GraspAttempt]
    ) -> NDArray[np.float64]:
        """
        :param object_name: The object the grasps were drawn for.
        :param model_index: The model they were drawn from; ``None`` for uniformly.
        :param attempts: Attempts on the object.
        :return: The density of each attempt's grasp under the distribution.
        """
        circuit = self.of(object_name, model_index)
        points = SurfaceGraspVariables(circuit).points(
            [attempt.grasp for attempt in attempts]
        )
        return circuit.likelihood(points)


@dataclass
class ImportanceWeights:
    """
    Weights that turn the attempts on each object into attempts with grasps drawn
    uniformly from what the object's statement allows: the uniform density over the
    density of the mixture of every distribution that drew grasps for the object, each
    weighted by how many attempts it drew.

    Each object keeps its share of the attempts.
    """

    weights: NDArray[np.float64]
    """
    The weight of each attempt.
    """

    @classmethod
    def of(
        cls,
        attempts: Dict[int, GraspAttempt],
        phases: List[SamplingPhase],
        densities: SamplingDensities,
    ) -> ImportanceWeights:
        """
        :param attempts: The attempts, by the number they were stored under.
        :param phases: The phases they were drawn in.
        :param densities: The distributions they were drawn from.
        :return: The weights of the attempts, in their order.
        """
        numbers = list(attempts)
        object_names = np.array([attempt.object_name for attempt in attempts.values()])
        weights = np.zeros(len(numbers))
        for object_name in np.unique(object_names):
            on_object = np.flatnonzero(object_names == object_name)
            object_attempts = [attempts[numbers[index]] for index in on_object]
            mixture = np.zeros(len(on_object))
            total = 0
            for phase in phases:
                count = sum(
                    phase.holds(numbers[index], object_name) for index in on_object
                )
                if count == 0:
                    continue
                mixture += count * densities.likelihood(
                    object_name, phase.model_index, object_attempts
                )
                total += count
            mixture /= max(total, 1)
            uniform = densities.likelihood(object_name, None, object_attempts)
            object_weights = np.divide(
                uniform, mixture, out=np.zeros_like(uniform), where=mixture > 0
            )
            object_weights *= len(on_object) / max(object_weights.sum(), 1e-300)
            weights[on_object] = object_weights
        return cls(weights=weights)

    @property
    def effective_sample_size(self) -> float:
        """
        How many unweighted attempts the weighted attempts are worth.
        """
        return float(self.weights.sum() ** 2 / np.sum(self.weights**2))


# %% distances


@dataclass
class Distance:
    """
    A Monte Carlo estimate of an L1 distance, between 0 for equal distributions and 2
    for disjoint ones.
    """

    value: float
    """
    The estimate.
    """

    standard_error: float
    """
    Its standard error.
    """

    @classmethod
    def joint_l1(
        cls,
        first: ProbabilisticCircuit,
        second: ProbabilisticCircuit,
        sample_size: int,
    ) -> Distance:
        """
        :param first: One distribution.
        :param second: Another, over the same variables.
        :param sample_size: How many samples are drawn from each.
        :return: The L1 distance of the two: twice the difference of the probabilities
            they give the region where the first is more likely.
        """
        first_samples = first.sample(sample_size)
        first_more_likely = first.likelihood(first_samples) > second.likelihood(
            first_samples
        )
        second_samples = second.sample(sample_size)
        second_in_region = first.likelihood(second_samples) > second.likelihood(
            second_samples
        )
        return cls(
            value=float(2 * (first_more_likely.mean() - second_in_region.mean())),
            standard_error=float(
                2
                * np.sqrt(
                    first_more_likely.var() / sample_size
                    + second_in_region.var() / sample_size
                )
            ),
        )

    @classmethod
    def conditional_l1(
        cls,
        first: ProbabilisticCircuit,
        second: ProbabilisticCircuit,
        uniform: ProbabilisticCircuit,
        sample_size: int,
    ) -> Distance:
        """
        :param first: One joint distribution over grasps and their results.
        :param second: Another, over the same variables.
        :param uniform: The uniform distribution over the grasps the statement allows.
        :param sample_size: How many samples are drawn from each joint distribution.
        :return: The L1 distance between the distributions of results given the grasp,
            averaged over grasps drawn from ``uniform``: the L1 distance of
            ``uniform(grasp) first(result | grasp)`` and ``uniform(grasp) second(result
            | grasp)``, estimated from each joint distribution's own samples with the
            weights ``uniform(grasp) / joint(grasp)``.
        """
        columns = SurfaceGraspVariables(first).columns_of_the_grasp
        marginals = {
            id(first): first.marginal(SurfaceGraspVariables(first).of_the_grasp),
            id(second): second.marginal(SurfaceGraspVariables(second).of_the_grasp),
        }

        def conditional(
            joint: ProbabilisticCircuit, samples: NDArray[np.float64]
        ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
            marginal = marginals[id(joint)].likelihood(samples[:, columns])
            density = np.divide(
                joint.likelihood(samples),
                marginal,
                out=np.zeros(len(samples)),
                where=marginal > 0,
            )
            return density, marginal

        def share_in_region(joint: ProbabilisticCircuit) -> Tuple[float, float]:
            samples = joint.sample(sample_size)
            first_conditional, first_marginal = conditional(first, samples)
            second_conditional, second_marginal = conditional(second, samples)
            own_marginal = first_marginal if joint is first else second_marginal
            weights = np.divide(
                uniform.likelihood(samples[:, columns]),
                own_marginal,
                out=np.zeros(len(samples)),
                where=own_marginal > 0,
            )
            terms = weights * (first_conditional > second_conditional)
            return float(terms.mean()), float(terms.var() / sample_size)

        first_share, first_variance = share_in_region(first)
        second_share, second_variance = share_in_region(second)
        return cls(
            value=2 * (first_share - second_share),
            standard_error=float(2 * np.sqrt(first_variance + second_variance)),
        )


# %% one round's measurements


@dataclass
class ModelPair:
    """
    The two models learned from one set of attempts: as they were drawn, and reweighted
    to uniformly drawn grasps.
    """

    unweighted: ProbabilisticCircuit
    """
    The model learned from the attempts as they were drawn.
    """

    weighted: ProbabilisticCircuit
    """
    The model learned from the attempts reweighted to uniformly drawn grasps.
    """


@dataclass
class Measurements:
    """
    How much the models changed from the previous round, next to how much learning alone
    changes them.
    """

    attempts: int
    """
    How many attempts the round's models were learned from.
    """

    effective_sample_size: float
    """
    How many unweighted attempts the reweighted attempts are worth.
    """

    conditional_l1: Optional[Distance]
    """
    The conditional L1 distance from the previous round's model; ``None`` in the first
    round.
    """

    conditional_floor: Distance
    """
    The conditional L1 distance between two models learned from resamples of the round's
    attempts.
    """

    weighted_joint_l1: Optional[Distance]
    """
    The weighted joint L1 distance from the previous round's model; ``None`` in the
    first round.
    """

    weighted_joint_floor: Distance
    """
    The weighted joint L1 distance between two models learned from reweighted resamples
    of the round's attempts.
    """

    unweighted_joint_l1: Optional[Distance]
    """
    The joint L1 distance from the previous round's model as the attempts were drawn;
    ``None`` in the first round.
    """

    @property
    def converged(self) -> bool:
        """
        Whether both distances are no larger than their floors.
        """
        if self.conditional_l1 is None or self.weighted_joint_l1 is None:
            return False
        return (
            self.conditional_l1.value <= self.conditional_floor.value
            and self.weighted_joint_l1.value <= self.weighted_joint_floor.value
        )


@dataclass
class ConvergenceMeter:
    """
    Learns the two models of each round and measures how much they changed.
    """

    task: GraspLearningTask
    """
    The task the models are learned for.
    """

    main_object: str
    """
    The object whose stated regions the conditional distance averages over.
    """

    densities: SamplingDensities
    """
    The distributions the attempts were drawn from.
    """

    learner: GraspModelLearner = field(default_factory=GraspModelLearner)
    """
    Learns the models.
    """

    sample_size: int = 5000
    """
    How many samples each distance is estimated from.
    """

    seed: int = 0
    """
    Seeds the resampling.
    """

    previous: Optional[ModelPair] = None
    """
    The models of the previous round; ``None`` before the first.
    """

    def circuit(self, attempts: List[GraspAttempt]) -> ProbabilisticCircuit:
        """
        :param attempts: Attempts of :attr:`task`.
        :return: The joint distribution over grasps and their results learned from
            them.
        """
        model = self.learner.learn(self.task, attempts)
        part = self.densities.parts[self.main_object]
        return model.model_registry().get_model(
            UnderspecifiedParameters(part.surface_grasp_statement())
        )

    @staticmethod
    def resampled(
        attempts: List[GraspAttempt],
        weights: Optional[NDArray[np.float64]],
        generator: np.random.Generator,
    ) -> List[GraspAttempt]:
        """
        :param attempts: Attempts.
        :param weights: How likely each is drawn; ``None`` draws them equally likely.
        :param generator: Draws the attempts.
        :return: As many attempts drawn from them with replacement.
        """
        probabilities = None if weights is None else weights / weights.sum()
        indices = generator.choice(len(attempts), len(attempts), p=probabilities)
        return [attempts[index] for index in indices]

    def measure(
        self,
        round_number: int,
        attempts: Dict[int, GraspAttempt],
        phases: List[SamplingPhase],
    ) -> Measurements:
        """
        :param round_number: Seeds the resampling.
        :param attempts: Every attempt the round's model is learned from, by the number
            it was stored under.
        :param phases: The phases the attempts were drawn in.
        :return: The round's measurements; the round's models become the previous ones.
        """
        generator = np.random.default_rng((self.seed, round_number))
        np.random.seed(self.seed * 1000 + round_number)
        weights = ImportanceWeights.of(attempts, phases, self.densities)
        listed = list(attempts.values())
        current = ModelPair(
            unweighted=self.circuit(listed),
            weighted=self.circuit(self.resampled(listed, weights.weights, generator)),
        )
        uniform = self.densities.of(self.main_object, None)
        previous = self.previous
        measurements = Measurements(
            attempts=len(listed),
            effective_sample_size=weights.effective_sample_size,
            conditional_l1=(
                None
                if previous is None
                else Distance.conditional_l1(
                    previous.unweighted, current.unweighted, uniform, self.sample_size
                )
            ),
            conditional_floor=Distance.conditional_l1(
                self.circuit(self.resampled(listed, None, generator)),
                self.circuit(self.resampled(listed, None, generator)),
                uniform,
                self.sample_size,
            ),
            weighted_joint_l1=(
                None
                if previous is None
                else Distance.joint_l1(
                    previous.weighted, current.weighted, self.sample_size
                )
            ),
            weighted_joint_floor=Distance.joint_l1(
                self.circuit(self.resampled(listed, weights.weights, generator)),
                self.circuit(self.resampled(listed, weights.weights, generator)),
                self.sample_size,
            ),
            unweighted_joint_l1=(
                None
                if previous is None
                else Distance.joint_l1(
                    previous.unweighted, current.unweighted, self.sample_size
                )
            ),
        )
        self.previous = current
        return measurements

"""
Choosing grasps by how likely a learned model expects them to lift the object, for where
the object stands.

Grasps drawn from a model that requires lifting follow ``P(grasp | lifted)``, which is
``P(lifted | grasp)`` weighted by how often a grasp was tried. A region that lifts the
object in two of three attempts therefore keeps being drawn as long as it was tried
often. Choosing among drawn grasps only those the model expects to lift the object
almost always removes that region.

The robot reaches the object from where it stands, so whether it reaches a grasp depends
on the grasp's direction seen from the robot, not from the object. Models over grasps
seen from the world are learned over grasps whose azimuth is turned by the object's yaw,
and a grasp chosen from one is turned back by the yaw of the object it is tried on.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from typing_extensions import List, Optional

from experiments.grasp_learning.grasp_variables import SurfaceGraspVariables
from experiments.grasp_learning.models import GraspModel, GraspModelLearner
from experiments.grasp_learning.records import GraspAttempt, GraspLearningTask
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)
from semantic_digital_twin.grasping.surface_grasp import (
    SurfaceGrasp,
    draw_surface_grasps,
)
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% turning grasps between the object's frame and the world's


def turned(grasp: SurfaceGrasp, yaw: float) -> SurfaceGrasp:
    """
    :param grasp: A grasp.
    :param yaw: The angle to turn it by about the vertical axis, in radians.
    :return: A copy of the grasp, its azimuth turned by ``yaw`` into ``[0, 2 pi)``.
    """
    result = copy.copy(grasp)
    result.azimuth = float(np.mod(grasp.azimuth + yaw, 2 * math.pi))
    return result


@dataclass
class WorldFrameLearner(GraspModelLearner):
    """
    Learns a grasp model over grasps seen from the world, the azimuth of each turned by
    the yaw of the object it was tried on.
    """

    def learn(
        self, task: GraspLearningTask, attempts: List[GraspAttempt]
    ) -> GraspModel:
        seen_from_world = [copy.copy(attempt) for attempt in attempts]
        for attempt in seen_from_world:
            attempt.grasp = turned(attempt.grasp, attempt.placement.yaw)
        return super().learn(task, seen_from_world)


# %% how likely a grasp lifts the object


@dataclass
class LiftProbability:
    """
    ``P(lifted | grasp)`` under a learned model, as ``P(lifted) p(grasp | lifted) /
    p(grasp)``.
    """

    model: GraspModel
    """
    The learned model.
    """

    part: HasGraspCandidates
    """
    The grasped part whose statement the model answers.
    """

    lift_probability: float = field(init=False)
    """
    How likely any grasp the model describes lifts the object.
    """

    lifting_grasps: ProbabilisticCircuit = field(init=False)
    """
    The distribution of the grasps that lift the object.
    """

    all_grasps: ProbabilisticCircuit = field(init=False)
    """
    The distribution of every grasp.
    """

    def __post_init__(self):
        parameters = UnderspecifiedParameters(
            self.part.surface_grasp_statement(require_lifting=True)
        )
        circuit = self.model.model_registry().get_model(parameters)
        grasp_variables = SurfaceGraspVariables(circuit).of_the_grasp
        lifting, self.lift_probability = circuit.conditional(
            parameters.conditioning_assignments_from_literal_values
        )
        self.lifting_grasps = lifting.marginal(grasp_variables)
        self.all_grasps = circuit.marginal(grasp_variables)

    def of(self, grasps: List[SurfaceGrasp]) -> NDArray[np.float64]:
        """
        :param grasps: Grasps, in the frame the model was learned in.
        :return: How likely each grasp lifts the object; 0 where the model has never
            seen a grasp.
        """
        points = SurfaceGraspVariables(self.all_grasps).points(grasps)
        everywhere = self.all_grasps.likelihood(points)
        return np.clip(
            np.divide(
                self.lift_probability * self.lifting_grasps.likelihood(points),
                everywhere,
                out=np.zeros(len(points)),
                where=everywhere > 0,
            ),
            0.0,
            1.0,
        )


# %% choosing


@dataclass
class GraspSelection:
    """
    Chooses, among grasps drawn from a model, one the model expects to lift the object.
    """

    minimum_lift_probability: float = 0.9
    """
    Only drawn grasps the model expects to lift the object at least this likely are
    chosen; 0 chooses every drawn grasp, as drawing from the model alone does.
    """

    candidates: int = 256
    """
    How many grasps are drawn from the model to choose one from.
    """

    fallback_share: float = 0.1
    """
    The share of the most likely candidates chosen from when none is likely enough.
    """

    def choose(
        self,
        part: HasGraspCandidates,
        model: GraspModel,
        probability: LiftProbability,
        generator: np.random.Generator,
    ) -> Optional[SurfaceGrasp]:
        """
        :param part: The grasped part.
        :param model: The model the grasps are drawn from.
        :param probability: How likely the model expects a grasp to lift the object.
        :param generator: Chooses among the likely grasps.
        :return: One grasp drawn from the model among those likely enough to lift the
            object, in the frame the model was learned in, without a result; ``None``
            if the model draws none.
        """
        drawn = draw_surface_grasps(
            part.surface_grasp_statement(require_lifting=True),
            self.candidates if self.minimum_lift_probability > 0 else 1,
            model.model_registry(),
        )
        if not drawn:
            return None
        if self.minimum_lift_probability > 0:
            likely = probability.of(drawn)
            chosen = np.flatnonzero(likely >= self.minimum_lift_probability)
            if len(chosen) == 0:
                count = max(1, round(self.fallback_share * len(drawn)))
                chosen = np.argsort(likely)[-count:]
            drawn = [drawn[index] for index in chosen]
        grasp = copy.copy(drawn[generator.integers(len(drawn))])
        grasp.result = None
        return grasp

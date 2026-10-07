"""
Trying grasps on an object to collect the data a model of grasps is learned from.

The grasps are drawn from the statement of where the object's annotation may be grasped,
through a model registry: uniformly within the stated regions at first, from a learned
model later. Each tried grasp is recorded together with what happened, so the records
hold the grasp's parameters and its result as plain numbers and truth values.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

from typing_extensions import Iterator, List, Optional

from krrood.parametrization.model_registries import ModelRegistry
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.surface_grasp import (
    GraspResult,
    SurfaceGrasp,
    SurfaceGraspStatement,
)
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% trying a grasp


class GraspTrier(ABC):
    """
    Tries grasps and tells what happened, for example a robot in a physics simulation or
    in the real world.
    """

    @abstractmethod
    def try_grasp(self, grasp: GraspCandidate) -> GraspResult:
        """
        :param grasp: The grasp to try.
        :return: What happened to the object.
        """


# %% the trials


@dataclass
class GraspTrialRecord:
    """
    One tried grasp as it is recorded.
    """

    grasped_part: str
    """
    The name of the annotation type of the part the grasp was placed on.
    """

    grasp: SurfaceGrasp
    """
    The grasp, with its result.
    """


@dataclass
class GraspTrials:
    """
    Grasps drawn from the statement of where an object may be grasped, each tried and
    given its result.

    The grasps are placed on the object's grasped part: its handle when it has one.
    """

    graspable: HasGraspCandidates
    """
    The object to grasp.
    """

    trier: GraspTrier
    """
    Tries each grasp.
    """

    number_of_trials: int = 20
    """
    How many grasps are drawn.
    """

    model_registry: Optional[ModelRegistry] = field(default=None)
    """
    Answers the statement; ``None`` draws uniformly within the stated regions.
    """

    @property
    def grasped_part(self) -> HasGraspCandidates:
        """
        :return: The part of the object the grasps are placed on.
        """
        return self.graspable.grasped_part()

    def drawn_grasps(self) -> List[SurfaceGrasp]:
        """
        :return: :attr:`number_of_trials` grasps answering the statement of where
            :attr:`grasped_part` may be grasped.
        """
        return SurfaceGraspStatement(self.grasped_part.surface_grasp_regions()).draw(
            self.number_of_trials, self.model_registry
        )

    def run(self) -> Iterator[GraspTrialRecord]:
        """
        Try every drawn grasp.

        :return: A record of each grasp tried, with its result, as they are tried. A
            grasp naming a point the grasped part's surface does not reach is skipped.
        """
        part = self.grasped_part
        for surface_grasp in self.drawn_grasps():
            if not surface_grasp.reaches_surface_of(part):
                continue
            surface_grasp.result = self.trier.try_grasp(
                surface_grasp.grasp_candidate(part)
            )
            yield GraspTrialRecord(
                grasped_part=type(part).__name__, grasp=surface_grasp
            )

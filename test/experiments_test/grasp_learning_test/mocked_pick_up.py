"""
Pick-ups judged without simulating them, so that learning can be tested in moments.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from typing_extensions import List, Optional

from experiments.grasp_learning.workers import (
    AttemptBatch,
    AttemptOutcome,
    IterationWorker,
    WorkerPool,
)
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.robots import ObjectPlacement
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.surface_grasp import GraspResult
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.spatial_types.spatial_types import AxisAngle

# %% a pick-up judged by the height of its grasp


@dataclass
class HeightJudgedPickUp(PickUpExperiment):
    """
    Lifts the object exactly when the grasp takes hold above a share of the height of
    the object's grasp surface, without simulating the attempt.
    """

    lifting_height: float = 0.5
    """
    The share of the grasp surface's height above which a grasp lifts the object.
    """

    def run(
        self,
        grasp: Optional[GraspCandidate] = None,
        video_path: Optional[Path] = None,
        placement: Optional[ObjectPlacement] = None,
    ) -> GraspResult:
        lowest, highest = self.scene.graspable.grasp_surface().bounds
        threshold = lowest[2] + self.lifting_height * (highest[2] - lowest[2])
        raised = bool(grasp.grasp_pose.to_np()[2, 3] > threshold)
        return GraspResult(
            object_raised=raised,
            motion_completed=True,
            object_displacement=Vector3(0.0, 0.0, 0.2 if raised else 0.0),
            object_rotation=AxisAngle(),
            translational_slip=Vector3(0.0, 0.0, 0.0 if raised else -0.2),
            rotational_slip=AxisAngle(),
        )


# %% workers in this process


@dataclass
class InProcessWorkerPool(WorkerPool):
    """
    Tries every batch in this process, with a worker whose pick-ups are judged by
    :class:`HeightJudgedPickUp`.
    """

    worker: IterationWorker = field(kw_only=True)
    """
    Tries the batches.
    """

    def __post_init__(self):
        for iteration_object in self.objects:
            self.worker.experiments[iteration_object.name] = HeightJudgedPickUp(
                scene=iteration_object.scene()
            )

    def run(self, batches: List[AttemptBatch]) -> List[AttemptOutcome]:
        return [
            outcome for batch in batches for outcome in self.worker.try_batch(batch)
        ]

"""
What is stored about grasping: each attempt to pick an object up, and the task it
belongs to.

Attempts are kept apart by task, the kind of object, the kind of part grasped on it and
the kind of gripper, so that each task's model is learned from its own attempts, and
each task can later become a column of a progressive probabilistic circuit.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from experiments.physical_pick_up.robots import ObjectPlacement
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% tasks


@dataclass
class GraspLearningTask:
    """
    What one grasp model is learned for: grasps placed on one kind of part of one kind
    of object, by one kind of gripper.
    """

    annotation_type: str
    """
    The name of the annotation type of the object, such as ``Mug``.
    """

    grasped_part: str
    """
    The name of the annotation type of the part the grasps are placed on, such as
    ``Handle``; the object's own type when it is grasped as a whole.
    """

    gripper: str
    """
    The name of the type of the gripper that grasps.
    """

    @classmethod
    def of_graspable(
        cls, graspable: HasGraspCandidates, gripper: str
    ) -> GraspLearningTask:
        """
        :param graspable: The object to grasp.
        :param gripper: The name of the type of the gripper that grasps.
        :return: The task of grasping the object's grasped part with that gripper.
        """
        return cls(
            annotation_type=type(graspable).__name__,
            grasped_part=type(graspable.grasped_part()).__name__,
            gripper=gripper,
        )


# %% attempts


class GraspSource(StrEnum):
    """
    Where the grasp of an attempt was drawn from.
    """

    STATED_REGIONS = "stated regions"
    """
    Uniformly from the regions the object's annotation states.
    """

    LEARNED_MODEL = "learned model"
    """
    From a learned model, asked for grasps that lift the object.
    """


@dataclass
class GraspAttempt:
    """
    One attempt of a robot to pick an object up by a surface grasp.
    """

    task: GraspLearningTask
    """
    The task the attempt belongs to.
    """

    robot: str
    """
    The name of the type of the robot that tried the grasp.
    """

    object_name: str
    """
    The name of the object's body, which tells which model of the object it was.
    """

    source: GraspSource
    """
    Where the grasp was drawn from.
    """

    placement: ObjectPlacement
    """
    Where the object stood.
    """

    grasp: SurfaceGrasp
    """
    The grasp, with its result.
    """

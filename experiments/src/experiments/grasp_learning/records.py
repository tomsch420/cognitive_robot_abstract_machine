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
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp

# %% tasks


@dataclass(frozen=True)
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
    def of_scene(cls, scene: PickUpScene) -> GraspLearningTask:
        """
        :param scene: The scene whose robot picks up the object in it.
        :return: The task the scene's attempts belong to.
        """
        return cls(
            annotation_type=type(scene.graspable).__name__,
            grasped_part=type(scene.graspable.grasped_part()).__name__,
            gripper=type(scene.arm.end_effector).__name__,
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

    annotation_type: str
    """
    See :attr:`GraspLearningTask.annotation_type`.
    """

    grasped_part: str
    """
    See :attr:`GraspLearningTask.grasped_part`.
    """

    gripper: str
    """
    See :attr:`GraspLearningTask.gripper`.
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

    @property
    def task(self) -> GraspLearningTask:
        """
        :return: The task this attempt belongs to.
        """
        return GraspLearningTask(
            annotation_type=self.annotation_type,
            grasped_part=self.grasped_part,
            gripper=self.gripper,
        )

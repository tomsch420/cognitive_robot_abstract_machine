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

from typing_extensions import Type

from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.robocasa_atomic_tasks.stored_runs import (
    RecordedPlan,
    SimulatedMoment,
)
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.robots.robot_parts import AbstractRobot, EndEffector
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% tasks


@dataclass
class GraspLearningTask:
    """
    What one grasp model is learned for: grasps placed on one kind of part of one kind
    of object, by one kind of gripper.
    """

    annotation_type: Type[HasGraspCandidates]
    """
    The annotation type of the object, such as :class:`Mug`.
    """

    grasped_part: Type[HasGraspCandidates]
    """
    The annotation type of the part the grasps are placed on, such as :class:`Handle`;
    the object's own type when it is grasped as a whole.
    """

    gripper: Type[EndEffector]
    """
    The type of the gripper that grasps.
    """

    @classmethod
    def of_graspable(
        cls, graspable: HasGraspCandidates, gripper: Type[EndEffector]
    ) -> GraspLearningTask:
        """
        :param graspable: The object to grasp.
        :param gripper: The type of the gripper that grasps.
        :return: The task of grasping the object's grasped part with that gripper.
        """
        return cls(
            annotation_type=type(graspable),
            grasped_part=type(graspable.grasped_part()),
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

    STARTING_MODEL = "starting model"
    """
    From a model learned from attempts on other objects, asked for grasps that lift
    the object.
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

    robot: Type[AbstractRobot]
    """
    The type of the robot that tried the grasp.
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


# %% attempts with what they left behind


@dataclass(kw_only=True)
class RecordedGraspAttempt:
    """
    An attempt performed by a plan in a physical simulation, with the plans it ran and
    the world's state as it moved.
    """

    attempt: GraspAttempt
    """
    The attempt, with its result.
    """

    degree_of_freedom_names: list[str]
    """
    The names of the world's degrees of freedom, in the order every moment lists their
    positions.
    """

    moments: list[SimulatedMoment]
    """
    The world's state through the attempt, in the order it passed.
    """

    plans: list[RecordedPlan]
    """
    The plans the robot performed in the attempt, in the order they began.
    """

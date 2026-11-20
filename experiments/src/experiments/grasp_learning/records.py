"""
What is stored about grasping: each attempt to pick an object up, and the task it
belongs to.

Attempts are kept apart by task, the kind of object, the kind of part grasped on it and
the kind of gripper, so that each task's model is learned from its own attempts, and
each task can later become a column of a progressive probabilistic circuit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from typing_extensions import Type

from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.robocasa_atomic_tasks.stored_runs import StoredTaskRun
from semantic_digital_twin.grasping.object_relative_grasp import GraspResult
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


# %% attempts with their runs


@dataclass(kw_only=True)
class RecordedGraspAttempt:
    """
    An attempt performed by plans in a physical simulation, with the run it was.
    """

    attempt: GraspAttempt
    """
    The attempt, with its result.
    """

    run: StoredTaskRun
    """
    The plans the robot performed in the attempt and the world's state as it moved.
    """


# %% whole pick-ups from a stored starting world


@dataclass(kw_only=True)
class StandOffset:
    """
    How far the base is moved from where it stands in the starting world, in the frame
    of the base as it stands there.
    """

    forward: float
    """
    How far the base moves along its x-axis, in metres.
    """

    sideways: float
    """
    How far the base moves along its y-axis, to its left, in metres.
    """

    turn: float
    """
    How far the base turns about its z-axis, counterclockwise, in radians.
    """


@dataclass(kw_only=True)
class PickUpParameters:
    """
    Everything a pick-up from the starting world varies by.
    """

    stand_offset: StandOffset
    """
    Where the base stands for the pick-up.
    """

    torso_height: float
    """
    The position of the torso's lift joint during the pick-up, in metres.
    """

    grasp: SurfaceGrasp
    """
    Where on the object the fingers close, and from which direction they come.
    """

    opening: float
    """
    How far the fingers open before they approach, as a fraction of how far they open
    at most.
    """

    closing_force: float
    """
    The most force each finger presses with, in newtons.
    """

    approach_clearance: float
    """
    How far outside the object's bounding box the gripper stops before it comes onto
    the grasp, in metres.
    """

    approach_velocity: float
    """
    How fast the tool frame comes down onto the grasp, in m/s.
    """

    settling_duration: float
    """
    How long the closed fingers hold still on the object before lifting, in seconds.
    """

    lift_velocity: float
    """
    How fast the tool frame leaves the grasp, in m/s.
    """

    def values_by_attribute_path(self) -> dict[str, float]:
        """
        :return: Every parameter, keyed by the path of attributes that leads to it from
            this object, such as ``stand_offset.forward``.
        """
        return {
            "stand_offset.forward": self.stand_offset.forward,
            "stand_offset.sideways": self.stand_offset.sideways,
            "stand_offset.turn": self.stand_offset.turn,
            "torso_height": self.torso_height,
            "grasp.yaw": self.grasp.yaw,
            "grasp.height": self.grasp.height,
            "grasp.depth": self.grasp.depth,
            "grasp.pitch": self.grasp.pitch,
            "grasp.roll": self.grasp.roll,
            "opening": self.opening,
            "closing_force": self.closing_force,
            "approach_clearance": self.approach_clearance,
            "approach_velocity": self.approach_velocity,
            "settling_duration": self.settling_duration,
            "lift_velocity": self.lift_velocity,
        }


class PickUpStage(StrEnum):
    """
    A part of a pick-up, in the order a pick-up goes through them.
    """

    STANDING = "standing"
    """
    Moving the base to where it stands for the pick-up.
    """

    APPROACHING = "approaching"
    """
    Opening the fingers and coming down onto the grasp.
    """

    CLOSING = "closing"
    """
    Closing the fingers on the object.
    """

    LEAVING = "leaving"
    """
    Going back along the way in to where the gripper started.
    """


@dataclass(kw_only=True)
class PickUpTrial:
    """
    One pick-up tried from a stored starting world, with what came of it.
    """

    starting_world_id: int
    """
    The database id of the world the trial started from.
    """

    parameters: PickUpParameters
    """
    What the pick-up was varied by.
    """

    result: GraspResult
    """
    What happened to the object.
    """

    failed_stage: PickUpStage | None
    """
    The part of the pick-up whose plan failed; ``None`` when every part went through.
    """

    run: StoredTaskRun
    """
    The plans the robot performed and the world's state as it moved.
    """


# %% letting go of a held object from a stored starting world


@dataclass(kw_only=True)
class ReleaseParameters:
    """
    Everything letting go of a held object that rests on its target varies by.
    """

    opening: float
    """
    How far the fingers open, as a fraction of how far apart they are when fully open.
    """

    opening_force: float
    """
    The most force each finger pushes with while it opens, in newtons.
    """

    settling_duration: float
    """
    How long the opened fingers hold still before the hand moves away, in seconds.
    """

    retreat_tilt: float
    """
    The angle between the direction the hand moves away in and the way back along its
    approach axis, in radians.
    """

    retreat_turn: float
    """
    Which way the direction the hand moves away in tilts, as an angle about the approach
    axis counterclockwise from the closing axis, in radians.
    """

    retreat_distance: float
    """
    How far the tool frame moves away, in metres.
    """

    retreat_velocity: float
    """
    How fast the tool frame moves away, in m/s.
    """

    def values_by_attribute_path(self) -> dict[str, float]:
        """
        :return: Every parameter, keyed by its attribute name.
        """
        return {
            "opening": self.opening,
            "opening_force": self.opening_force,
            "settling_duration": self.settling_duration,
            "retreat_tilt": self.retreat_tilt,
            "retreat_turn": self.retreat_turn,
            "retreat_distance": self.retreat_distance,
            "retreat_velocity": self.retreat_velocity,
        }

    def retreat_direction(
        self, *, approach_axis: np.ndarray, closing_axis: np.ndarray
    ) -> np.ndarray:
        """
        :param approach_axis: The unit axis the gripper approaches along.
        :param closing_axis: The unit axis its fingers close along, perpendicular to
            ``approach_axis`` and in the same frame.
        :return: The unit direction the hand moves away in, in that frame.
        """
        tilted_towards = math.cos(self.retreat_turn) * closing_axis + math.sin(
            self.retreat_turn
        ) * np.cross(approach_axis, closing_axis)
        return (
            -math.cos(self.retreat_tilt) * approach_axis
            + math.sin(self.retreat_tilt) * tilted_towards
        )


class ReleaseStage(StrEnum):
    """
    A part of letting go, in the order letting go goes through them.
    """

    OPENING = "opening"
    """
    Opening the fingers.
    """

    RETREATING = "retreating"
    """
    Moving the opened hand away from the object.
    """


@dataclass(kw_only=True)
class ReleaseResult:
    """
    What came of letting go of an object.
    """

    finger_separation_after_opening: float
    """
    How far apart the fingers were once they opened, in metres.
    """

    fingers_touched_the_object_after_opening: bool
    """
    Whether a finger still touched the object once the fingers opened.
    """

    object_carried_along: float
    """
    How far the object moved while the hand moved away and the object came to rest, in
    metres.
    """

    fingers_touched_the_object_at_the_end: bool
    """
    Whether a finger still touched the object once the hand had moved away.
    """

    accomplished: bool
    """
    Whether RoboCasa judged the task accomplished at the end.
    """


@dataclass(kw_only=True)
class ReleaseTrial:
    """
    One way of letting go tried from a stored starting world, with what came of it.
    """

    starting_world_id: int
    """
    The database id of the world the trial started from.
    """

    parameters: ReleaseParameters
    """
    What letting go was varied by.
    """

    result: ReleaseResult
    """
    What happened to the object.
    """

    failed_stage: ReleaseStage | None
    """
    The part of letting go whose plan failed; ``None`` when every part went through.
    """

    run: StoredTaskRun
    """
    The plans the robot performed and the world's state as it moved.
    """

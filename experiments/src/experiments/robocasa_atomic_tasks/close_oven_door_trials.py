"""
Trials of the PandaOmron closing RoboCasa's oven door by its handle with
:class:`~experiments.robocasa_atomic_tasks.skills.CloseByTheHandleAction`, in MuJoCo, to learn
from where and how it succeeds.
"""

from __future__ import annotations

import logging
import math
from contextlib import contextmanager
from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.robot_plans.actions.core.navigation import NavigateAction
from experiments.robocasa_atomic_tasks.doors import CloseOven
from experiments.robocasa_atomic_tasks.door_handles import RoboCasaDoorHandleSplitter
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.skills import CloseByTheHandleAction
from krrood.bounds import Bounds
from krrood.entity_query_language.factories import ConditionType, or_
from krrood.entity_query_language.query.match import Match
from krrood.ormatic.data_access_objects.dao import DataAccessObject
from krrood.parametrization.trials.campaigns import TrialPerformer, TrialSetup
from krrood.parametrization.trials.database import DatabaseTrialStore
from krrood.parametrization.trials.parameter_spaces import ParameterSpace
from krrood.parametrization.trials.records import (
    Trial,
    TrialMeasurements,
    TrialParameters,
)
from krrood_spatial_math.spatial_types import Point2, Pose2D
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)

if TYPE_CHECKING:
    from semantic_digital_twin.world import World

logger = logging.getLogger(__name__)

# %% what a trial varies by


class HandleApproach(Enum):
    """
    The way the gripper comes at the handle, as the direction it moves in the frame of
    the door, whose y axis points into the closed oven and whose z axis points up.
    """

    FROM_THE_FRONT = (0.0, 1.0, 0.0)
    """
    Towards the door, from the room.
    """

    FROM_BELOW = (0.0, 0.0, 1.0)
    """
    Upwards, from below the handle.
    """

    FROM_BEHIND = (0.0, -1.0, 0.0)
    """
    Away from the door, from between the handle and the door.
    """

    FROM_ABOVE = (0.0, 0.0, -1.0)
    """
    Downwards, from above the handle.
    """


class BaseMotion(Enum):
    """
    Whether the base moves while the arm closes the door.
    """

    STANDING_STILL = "standing still"
    """
    Only the arm moves.
    """

    MOVING_WITH_THE_ARM = "moving with the arm"
    """
    The base moves together with the arm, as one body.
    """


@dataclass(kw_only=True)
class StandBeforeTheHandle:
    """
    Where the base stands before it closes the door, relative to the middle of the
    handle of the open door, as seen by a robot facing the oven.
    """

    sideways: float
    """
    How far to the right of the handle's middle the base stands, in meters.
    """

    distance: float
    """
    How far back from the handle's middle the base stands, in meters.
    """

    turn: float
    """
    How far the base is turned to the left of facing the oven, in radians.
    """


@dataclass(kw_only=True)
class CloseOvenDoorParameters(TrialParameters):
    """
    Everything a trial of closing the oven door varies by.
    """

    stand: StandBeforeTheHandle
    """
    Where the base stands while it closes the door.
    """

    approach: HandleApproach
    """
    How the gripper comes at the handle.
    """

    base_motion: BaseMotion
    """
    Whether the base moves while the arm closes the door.
    """


@dataclass(kw_only=True)
class CloseOvenDoorSpace(ParameterSpace[CloseOvenDoorParameters]):
    """
    The trials of closing the oven door from in front of it.
    """

    sideways: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=-0.3, upper=0.3)
    )
    """
    The values of :attr:`StandBeforeTheHandle.sideways`.
    """

    distance: Bounds[float] = field(
        default_factory=lambda: Bounds(lower=0.35, upper=0.75)
    )
    """
    The values of :attr:`StandBeforeTheHandle.distance`.
    """

    turn: Bounds[float] = field(default_factory=lambda: Bounds(lower=-0.4, upper=0.4))
    """
    The values of :attr:`StandBeforeTheHandle.turn`.
    """

    def conditions(
        self, *, parameters: Match[CloseOvenDoorParameters]
    ) -> list[ConditionType]:
        return [
            *self.conditions_within(
                value=parameters.stand.sideways, bounds=self.sideways
            ),
            *self.conditions_within(
                value=parameters.stand.distance, bounds=self.distance
            ),
            *self.conditions_within(value=parameters.stand.turn, bounds=self.turn),
            or_(*[parameters.approach == approach for approach in HandleApproach]),
            or_(*[parameters.base_motion == motion for motion in BaseMotion]),
        ]


# %% what a trial measures


class CloseOvenDoorStage(Enum):
    """
    The stages of a trial of closing the oven door.
    """

    DRIVING_TO_THE_STAND = "driving to the stand"
    """
    The base drives to where it closes the door from.
    """

    CLOSING_THE_DOOR = "closing the door"
    """
    The arm takes hold of the handle, swings the door shut and lets go.
    """


@dataclass(kw_only=True)
class CloseOvenDoorMeasurements(TrialMeasurements):
    """
    What is measured while the oven door is closed.
    """

    stand_position_error: float
    """
    How far the base stopped from the stand it drove to, in meters.
    """

    stand_turn_error: float
    """
    How far the base stopped turned from the stand it drove to, in radians.
    """

    door_angle: float
    """
    Where the door's joint ended up, in radians; ``0`` is shut.
    """

    failed_stage: CloseOvenDoorStage | None = None
    """
    The stage whose plan failed; ``None`` if every plan ended as planned.
    """

    failure: str | None = None
    """
    The name of the failure the plan of :attr:`failed_stage` ended with.
    """


@dataclass(kw_only=True)
class CloseOvenDoorTrial(Trial[CloseOvenDoorParameters, CloseOvenDoorMeasurements]):
    """
    One try of closing the oven door, succeeded when RoboCasa judges it shut.
    """


# %% closing the door


@dataclass(kw_only=True)
class StandOnTheFloor:
    """
    Where the base stands on the floor and which way it faces.
    """

    position: NDArray[np.float64]
    """
    The x and y of the base in the world, in meters.
    """

    heading: float
    """
    The way the base faces, as an angle about the vertical in the world, in radians.
    """

    @classmethod
    def from_base_pose(cls, *, base_pose: NDArray[np.float64]) -> StandOnTheFloor:
        """
        :param base_pose: The pose of the base in the world, as a homogeneous matrix.
        :return: Where the base stands in that pose.
        """
        return cls(
            position=base_pose[:2, 3].copy(),
            heading=float(math.atan2(base_pose[1, 0], base_pose[0, 0])),
        )

    def to_pose(self, *, world: World) -> Pose2D:
        """
        :param world: The world the base stands in.
        :return: The stand as a pose in the world's root frame.
        """
        return Pose2D(
            position=Point2(
                x=float(self.position[0]),
                y=float(self.position[1]),
                reference_frame=world.root,
            ),
            yaw=self.heading,
            reference_frame=world.root,
        )


@dataclass(kw_only=True)
class CloseOvenDoorPerformer(
    TrialPerformer[CloseOvenDoorParameters, CloseOvenDoorTrial]
):
    """
    Closes the oven door in a physically simulated episode, each time from the same
    moment of the simulation.
    """

    episode: RoboCasaEpisode
    """
    The episode, physically simulated.
    """

    handle: Handle
    """
    The handle of the oven door.
    """

    starting_moment: MujocoSimulationSnapshot
    """
    The moment of the simulation every trial starts from.
    """

    start: StandOnTheFloor
    """
    Where the base stands at :attr:`starting_moment`, facing the oven.
    """

    handle_middle: NDArray[np.float64]
    """
    The x and y of the middle of the handle at :attr:`starting_moment`, in meters.
    """

    def perform(self, *, parameters: CloseOvenDoorParameters) -> CloseOvenDoorTrial:
        self.episode.simulation.restore_snapshot(snapshot=self.starting_moment)
        stand = self.stand_on_the_floor(stand=parameters.stand)
        self.episode.robot.mobile_base.full_body_controlled = False
        driving = self.episode.run(
            [NavigateAction(planar_goal_pose=stand.to_pose(world=self.episode.world))]
        )
        reached = StandOnTheFloor.from_base_pose(
            base_pose=self.episode.robot.root.global_pose.to_np()
        )
        if driving.failure is not None:
            return self.ended_trial(
                parameters=parameters,
                stand=stand,
                reached=reached,
                failed_stage=CloseOvenDoorStage.DRIVING_TO_THE_STAND,
                failure=driving.failure,
            )
        self.episode.robot.mobile_base.full_body_controlled = (
            parameters.base_motion is BaseMotion.MOVING_WITH_THE_ARM
        )
        closing = self.episode.run(
            [
                CloseByTheHandleAction(
                    grasp_candidate=self.grasp(approach=parameters.approach),
                    arm=self.episode.arm,
                    closed_joint_position=0.0,
                )
            ]
        )
        self.episode.let_physics_run()
        return self.ended_trial(
            parameters=parameters,
            stand=stand,
            reached=reached,
            failed_stage=(
                None if closing.failure is None else CloseOvenDoorStage.CLOSING_THE_DOOR
            ),
            failure=closing.failure,
        )

    def stand_on_the_floor(self, *, stand: StandBeforeTheHandle) -> StandOnTheFloor:
        """
        :param stand: Where the base stands, relative to the handle.
        :return: Where that is on the floor; facing the oven is the way the base faces
            at :attr:`start`.
        """
        forward = np.array([math.cos(self.start.heading), math.sin(self.start.heading)])
        right = np.array([forward[1], -forward[0]])
        return StandOnTheFloor(
            position=self.handle_middle
            - stand.distance * forward
            + stand.sideways * right,
            heading=self.start.heading + stand.turn,
        )

    def grasp(self, *, approach: HandleApproach) -> GraspCandidate:
        """
        :param approach: How the gripper comes at the handle.
        :return: The grasp of the handle that comes at it most nearly that way.
        """
        door_T_world = np.linalg.inv(
            self.handle.root.parent_connection.parent.global_pose.to_np()
        )
        return max(
            self.handle.grasp_candidates(),
            key=lambda grasp: float(
                (door_T_world @ grasp.world_T_grasp.to_np())[:3, 0]
                @ np.array(approach.value)
            ),
        )

    def ended_trial(
        self,
        *,
        parameters: CloseOvenDoorParameters,
        stand: StandOnTheFloor,
        reached: StandOnTheFloor,
        failed_stage: CloseOvenDoorStage | None,
        failure: Exception | None,
    ) -> CloseOvenDoorTrial:
        """
        :param parameters: What the trial varied by.
        :param stand: Where the base drove to.
        :param reached: Where the base stopped driving.
        :param failed_stage: The stage whose plan failed, if any.
        :param failure: What that plan failed with, if anything.
        :return: The trial, its door measured as the world stands now.
        """
        door = self.handle.root.parent_connection.parent
        trial = CloseOvenDoorTrial(
            parameters=parameters,
            measurements=CloseOvenDoorMeasurements(
                stand_position_error=float(
                    np.linalg.norm(reached.position - stand.position)
                ),
                stand_turn_error=float(
                    math.remainder(reached.heading - stand.heading, math.tau)
                ),
                door_angle=float(door.parent_connection.position),
                failed_stage=failed_stage,
                failure=None if failure is None else type(failure).__name__,
            ),
            succeeded=self.episode.judge.pass_judgement().accomplished,
        )
        logger.info("Trial %s", trial)
        return trial


# %% the campaign's setup


@dataclass(kw_only=True)
class CloseOvenDoorTrialsInADatabase(DatabaseTrialStore[CloseOvenDoorTrial]):
    """
    Keeps trials of closing the oven door in a database of the experiments' ORM
    interface.
    """

    @property
    def data_access_object_class(self) -> type[DataAccessObject]:
        from experiments.orm.ormatic_interface import CloseOvenDoorTrialDAO

        return CloseOvenDoorTrialDAO


@dataclass(kw_only=True)
class CloseOvenDoorSetup(TrialSetup[CloseOvenDoorParameters, CloseOvenDoorTrial]):
    """
    The PandaOmron in RoboCasa's CloseOven episode, the oven door's handle split off into
    a body of its own, physically simulated in MuJoCo.
    """

    database_url: str
    """
    Where the trials are kept.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the episode with.
    """

    @contextmanager
    def performing(self) -> Iterator[CloseOvenDoorPerformer]:
        task = CloseOven(robot=RobotModel.PANDA_OMRON, physics=True, seed=self.seed)
        episode = task.compose()
        door_joint_names = task.joint_names(episode)
        (door,) = [
            door
            for door in episode.world.get_semantic_annotations_by_type(Door)
            if door.root.parent_connection.name.name in door_joint_names
        ]
        handle = RoboCasaDoorHandleSplitter().split_handle_off(door=door)
        with episode.executed(collision_avoidance=True, physics=True):
            episode.let_physics_run()
            handle_box = episode.bounds.of_body(handle.root)
            yield CloseOvenDoorPerformer(
                episode=episode,
                handle=handle,
                starting_moment=episode.simulation.take_snapshot(),
                start=StandOnTheFloor.from_base_pose(
                    base_pose=episode.robot.root.global_pose.to_np()
                ),
                handle_middle=((handle_box.lower + handle_box.upper) / 2)[:2],
            )

    def create_store(self) -> CloseOvenDoorTrialsInADatabase:
        return CloseOvenDoorTrialsInADatabase(database_url=self.database_url)

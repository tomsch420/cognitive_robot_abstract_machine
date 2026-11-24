"""
What trials of the PandaOmron moving RoboCasa's oven door by its handle, in MuJoCo, have
in common: where the base stands, how the gripper comes at the handle and the scene
every trial starts from.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING, Iterator

from coraplex.robot_plans.actions.core.navigation import NavigateAction
from experiments.robocasa_atomic_tasks.door_handles import RoboCasaDoorHandleSplitter
from experiments.robocasa_atomic_tasks.doors import ManipulateFixtureDoors
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from krrood.parametrization.trials.campaigns import TrialPerformer, TrialSetup
from krrood.parametrization.trials.records import ParametersTypeVar, TrialTypeVar
from krrood_spatial_math.spatial_types import Point2, Pose2D
from semantic_digital_twin.adapters.multi_sim import MujocoSimulationSnapshot
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)

if TYPE_CHECKING:
    from coraplex.plans.executors import PlanOutcome
    from semantic_digital_twin.world import World
    from semantic_digital_twin.world_description.world_entity import Body

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
    Whether the base moves while the arm moves the door.
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
    Where the base stands before it moves the door, relative to the middle of the
    handle at the start of the trial, as seen by a robot facing the oven.
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

    def stand_on_the_floor(
        self, *, handle_middle: NDArray[np.float64], heading_facing_the_oven: float
    ) -> StandOnTheFloor:
        """
        :param handle_middle: The x and y of the middle of the handle, in meters.
        :param heading_facing_the_oven: The heading, about the vertical in the world, of
            a base facing the oven, in radians.
        :return: Where this stand is on the floor.
        """
        forward = np.array(
            [math.cos(heading_facing_the_oven), math.sin(heading_facing_the_oven)]
        )
        right = np.array([forward[1], -forward[0]])
        return StandOnTheFloor(
            position=handle_middle - self.distance * forward + self.sideways * right,
            heading=heading_facing_the_oven + self.turn,
        )


# %% where the base stands


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

    def position_error(self, *, reached: StandOnTheFloor) -> float:
        """
        :param reached: Where the base stopped.
        :return: How far, in meters, it stopped from this stand.
        """
        return float(np.linalg.norm(reached.position - self.position))

    def turn_error(self, *, reached: StandOnTheFloor) -> float:
        """
        :param reached: Where the base stopped.
        :return: How far, in radians, it stopped turned from this stand, to the left
            positive.
        """
        return float(math.remainder(reached.heading - self.heading, math.tau))


# %% the scene a trial starts from


@dataclass(kw_only=True)
class OvenDoorScene:
    """
    The physically simulated episode a trial with the oven door starts from.
    """

    episode: RoboCasaEpisode
    """
    The episode, physically simulated.
    """

    handle: Handle
    """
    The handle of the oven door, split off into a body of its own.
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

    @property
    def door(self) -> Body:
        """
        The door the handle is mounted on.
        """
        return self.handle.root.parent_connection.parent

    def grasp(self, *, approach: HandleApproach) -> GraspCandidate:
        """
        :param approach: How the gripper comes at the handle.
        :return: The grasp of the handle that comes at it most nearly that way.
        """
        door_T_world = np.linalg.inv(self.door.global_pose.to_np())
        return max(
            self.handle.grasp_candidates(),
            key=lambda grasp: float(
                (door_T_world @ grasp.world_T_grasp.to_np())[:3, 0]
                @ np.array(approach.value)
            ),
        )

    def drive_to(self, *, stand: StandOnTheFloor) -> PlanOutcome:
        """
        Restore :attr:`starting_moment` and drive the base, alone, to a stand.

        :param stand: Where the base drives to.
        :return: How the drive ended.
        """
        self.episode.simulation.restore_snapshot(snapshot=self.starting_moment)
        self.episode.robot.mobile_base.full_body_controlled = False
        return self.episode.run(
            [NavigateAction(planar_goal_pose=stand.to_pose(world=self.episode.world))]
        )

    def where_the_base_stands(self) -> StandOnTheFloor:
        """
        :return: Where the base stands now.
        """
        return StandOnTheFloor.from_base_pose(
            base_pose=self.episode.robot.root.global_pose.to_np()
        )


@dataclass(kw_only=True)
class OvenDoorSetup(TrialSetup[ParametersTypeVar, TrialTypeVar], ABC):
    """
    The PandaOmron in a RoboCasa episode with the oven, the oven door's handle split off
    into a body of its own, physically simulated in MuJoCo.
    """

    database_url: str
    """
    Where the trials are kept.
    """

    seed: int = 0
    """
    The seed RoboCasa samples the episode with.
    """

    @abstractmethod
    def create_task(self) -> ManipulateFixtureDoors:
        """
        :return: A new task of the oven door, with the PandaOmron in physics, sampled with
            :attr:`seed`.
        """

    @abstractmethod
    def create_performer(
        self, *, scene: OvenDoorScene
    ) -> TrialPerformer[ParametersTypeVar, TrialTypeVar]:
        """
        :param scene: The scene every trial starts from.
        :return: A new performer of the trials in that scene.
        """

    @contextmanager
    def performing(self) -> Iterator[TrialPerformer[ParametersTypeVar, TrialTypeVar]]:
        task = self.create_task()
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
            yield self.create_performer(
                scene=OvenDoorScene(
                    episode=episode,
                    handle=handle,
                    starting_moment=episode.simulation.take_snapshot(),
                    start=StandOnTheFloor.from_base_pose(
                        base_pose=episode.robot.root.global_pose.to_np()
                    ),
                    handle_middle=((handle_box.lower + handle_box.upper) / 2)[:2],
                )
            )

"""
Doors hinged at their bottom edge, such as an oven's: where the base stands before the
door's handle, how the gripper comes at the handle, and letting the door down by it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np
from numpy.typing import NDArray
from typing_extensions import Self

from coraplex.robot_plans.actions.base import SequentialAction
from coraplex.robot_plans.actions.core.navigation import NavigateAction
from cramph.node import StatechartNode
from experiments.robocasa_atomic_tasks.skills import OpenByTheHandleAction
from krrood.patterns.classproperty import classproperty
from krrood_spatial_math.spatial_types import Point2, Pose2D
from semantic_digital_twin.collision_checking.collision_matrix import CollisionRule
from semantic_digital_twin.collision_checking.collision_rules import (
    AllowCollisionBetweenGroups,
)
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.robots.robot_parts import Arm
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

# %% how the gripper comes at the handle


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

    def closest_grasp(self, *, handle: Handle) -> GraspCandidate:
        """
        :param handle: The handle of a door, fixed to the door.
        :return: The grasp of the handle that comes at it most nearly this way.
        """
        door = handle.root.parent_connection.parent
        door_T_world = np.linalg.inv(door.global_pose.to_np())
        return max(
            handle.grasp_candidates(),
            key=lambda grasp: float(
                (door_T_world @ grasp.world_T_grasp.to_np())[:3, 0]
                @ np.array(self.value)
            ),
        )


# %% where the base stands


@dataclass(kw_only=True)
class StandBeforeTheHandle:
    """
    Where the base stands before it moves the door, relative to the middle of the
    handle, as seen by a robot facing the oven.
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

    @classproperty
    def left_of_the_handle(cls) -> StandBeforeTheHandle:
        """
        The stand 0.45 m left of the handle's middle and 0.6 m back, turned 0.1 rad to
        the left, from which the PandaOmron lets the oven door down.
        """
        return cls(sideways=-0.45, distance=0.6, turn=0.1)

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

    def stand_before(self, *, handle: Handle) -> StandOnTheFloor:
        """
        :param handle: The handle of a door swung open by less than a right angle, fixed
            to the door.
        :return: Where this stand is on the floor before that handle as it is now.
        """
        world = handle.root._world
        door = handle.root.parent_connection.parent
        into_the_oven = door.global_pose.to_np()[:2, 1]
        handle_box = (
            handle.root.collision.as_bounding_box_collection_in_frame(world.root)
            .bounding_box()
            .to_array_bounds()
        )
        return self.stand_on_the_floor(
            handle_middle=((handle_box.lower + handle_box.upper) / 2)[:2],
            heading_facing_the_oven=math.atan2(into_the_oven[1], into_the_oven[0]),
        )


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


# %% letting the door down


@dataclass(eq=False, repr=False, kw_only=True)
class LetTheDoorDownByItsHandleAction(SequentialAction):
    """
    Drives the base to a stand before the handle of a door hinged at its bottom edge,
    takes the handle from the front, pulls the door all the way down and lets go of it.
    The two links of the arm the hand hangs from may touch the door and its handle.

    .. note:: The base should move with the arm, as the robot's full body controlled.
    """

    navigate: NavigateAction
    """
    The step to the stand before the handle.
    """

    pull: OpenByTheHandleAction
    """
    The step that takes the handle, pulls the door down and lets go.
    """

    @classmethod
    def from_handle(
        cls,
        *,
        handle: Handle,
        arm: Arm,
        stand: StandBeforeTheHandle | None = None,
        release_clearance: float = 0.12,
    ) -> Self:
        """
        :param handle: The handle of the closed door, fixed to the door.
        :param arm: The arm that pulls the door down.
        :param stand: Where the base stands before the handle;
            :attr:`StandBeforeTheHandle.left_of_the_handle` if not given.
        :param release_clearance: See :attr:`OpenByTheHandleAction.release_clearance`.
        :return: The action letting the door down from that stand.
        """
        stand = StandBeforeTheHandle.left_of_the_handle if stand is None else stand
        return cls(
            navigate=NavigateAction(
                planar_goal_pose=stand.stand_before(handle=handle).to_pose(
                    world=handle.root._world
                )
            ),
            pull=OpenByTheHandleAction(
                grasp_candidate=HandleApproach.FROM_THE_FRONT.closest_grasp(
                    handle=handle
                ),
                arm=arm,
                release_clearance=release_clearance,
            ),
        )

    @staticmethod
    def wrist_allowance(*, arm: Arm, handle_body: Body) -> AllowCollisionBetweenGroups:
        """
        :param arm: The arm that pulls the door down.
        :param handle_body: The body of the door's handle, fixed to the door.
        :return: An allowance for the two links of the arm the hand hangs from to touch
            the door and its handle, which they come close to as the door swings down.
        """
        forearm_end = arm.end_effector.root.parent_connection.parent
        return AllowCollisionBetweenGroups(
            body_group_a=[forearm_end, forearm_end.parent_connection.parent],
            body_group_b=[handle_body.parent_connection.parent, handle_body],
        )

    def create_temporary_collision_rules(self) -> list[CollisionRule]:
        """
        :return: The allowance of :meth:`wrist_allowance`.
        """
        return [
            self.wrist_allowance(
                arm=self.pull.arm, handle_body=self.pull.grasp_candidate.graspable.root
            )
        ]

    def create_steps(self) -> list[StatechartNode]:
        return [self.navigate, self.pull]

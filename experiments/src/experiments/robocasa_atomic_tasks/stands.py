"""
Where the robot may stand: clear of the kitchen by the distances collision avoidance
keeps, and in front of what it reaches for.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field

import numpy as np

from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Pose,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


# %% how close the robot comes to the kitchen


@dataclass
class BodiesTooClose:
    """
    A body of the robot and another body closer to it than collision avoidance lets any
    motion bring them.
    """

    robot_body: Body
    """
    The body of the robot.
    """

    other: Body
    """
    The other body, of the kitchen or of the robot.
    """


@dataclass
class RobotClearance:
    """
    How far the robot keeps from the kitchen and from itself where it stands now.
    """

    world: World
    """
    The world the robot stands in.
    """

    robot: AbstractRobot
    """
    The robot.
    """

    def keeps_clear(
        self, base_margin: float = 0.03, arm_margin: float | None = None
    ) -> bool:
        """
        :param base_margin: How far beyond the distance collision avoidance never lets
            it come closer than the robot's base has to keep, in metres.
        :param arm_margin: How far beyond that distance the rest of the robot has to
            keep, in metres; ``None`` for the distance collision avoidance keeps it at.
        :return: Whether the robot keeps those distances from everything else and
            from its own other bodies.
        """
        collision_manager = self.world.collision_manager
        own = set(self.robot.bodies_with_collision)
        base = self.robot.mobile_base.root

        def required_distance(body_a: Body, body_b: Body) -> float:
            violated = collision_manager.get_violated_distance(body_a, body_b)
            if base in (body_a, body_b):
                return violated + base_margin
            if arm_margin is not None:
                return violated + arm_margin
            return collision_manager.get_buffer_zone_distance(body_a, body_b)

        return not any(
            contact.distance < required_distance(contact.body_a, contact.body_b)
            for contact in collision_manager.compute_collisions().contacts
            if contact.body_a in own or contact.body_b in own
        )

    def bodies_too_close(self, margin: float = 0.005) -> list[BodiesTooClose]:
        """
        :param margin: How far beyond the distance collision avoidance never lets them
            come closer than two bodies count as too close, in metres.
        :return: The pairs of a body of the robot and another body closer than
            collision avoidance lets any motion bring them.
        """
        collision_manager = self.world.collision_manager
        own = set(self.robot.bodies_with_collision)
        pairs = []
        for contact in collision_manager.compute_collisions().contacts:
            body_a, body_b = contact.body_a, contact.body_b
            if body_a not in own:
                if body_b not in own:
                    continue
                body_a, body_b = body_b, body_a
            violated = collision_manager.get_violated_distance(body_a, body_b)
            if contact.distance < violated + margin:
                pairs.append(BodiesTooClose(robot_body=body_a, other=body_b))
        return pairs


# %% where to stand


@dataclass
class StandCandidates:
    """
    The stands tried in front of a point, relative to it.
    """

    distances: tuple[float, ...] = (0.75, 0.65, 0.85, 0.55, 0.95, 1.05)
    """
    How far in front of the point the base may stand, in metres, best first.
    """

    offsets_to_the_left: tuple[float, ...] = (0.2, 0.1, 0.3, 0.0, 0.4, -0.1, -0.2)
    """
    How far to the robot's left of the point the base may stand, in metres, best
    first.
    """

    turns: tuple[float, ...] = (0.0, 0.4, -0.4)
    """
    By how much, in radians, the direction the stand lies in may be turned about the
    point away from the direction opposite to the one RoboCasa's start pose faces, best
    first.
    """

    number: int = 12
    """
    How many stands are returned at most.
    """


@dataclass
class StandFinder:
    """
    Finds where the robot may stand to reach a point with its right arm: in front of the
    point, facing the kitchen the way RoboCasa starts its robot facing it, a little to
    the left so that the point lies ahead of the right shoulder, and clear of the
    kitchen.
    """

    world: World
    """
    The world the robot stands in.
    """

    robot: AbstractRobot
    """
    The robot.
    """

    start_pose: HomogeneousTransformationMatrix
    """
    Where RoboCasa starts its own robot.
    """

    clearance: RobotClearance
    """
    How far the robot keeps from the kitchen.
    """

    arm_margin: float = 0.05
    """
    How far beyond the distance collision avoidance never lets them come closer than
    the arms have to keep from the kitchen where the robot is to stand, in metres.
    """

    candidates: StandCandidates = field(default_factory=StandCandidates)
    """
    The stands tried in front of a point.
    """

    @property
    def kitchen_direction(self) -> np.ndarray:
        """
        The horizontal direction RoboCasa's start pose faces the kitchen in.
        """
        facing = self.start_pose.to_np()[:3, 0]
        facing[2] = 0.0
        return facing / np.linalg.norm(facing)

    def approach_towards(self, point: np.ndarray) -> np.ndarray:
        """
        :param point: A point in the kitchen.
        :return: The levelled direction from the robot's base towards the point.
        """
        base = self.robot.root.global_pose.to_np()[:3, 3]
        direction = np.array(point[:2]) - base[:2]
        return np.append(direction / np.linalg.norm(direction), 0.0)

    def stands_facing(self, point: np.ndarray) -> list[Pose]:
        """
        :param point: The point to reach, in the world's root frame.
        :return: The stands among :attr:`candidates` where the robot keeps clear of the
            kitchen, those deviating least from the best distance, offset and turn
            first.
        """
        candidates = self.candidates
        start_facing = self.start_pose.to_np()[:2, 0]
        base_angle = math.atan2(-start_facing[1], -start_facing[0])
        drive = self.robot.root.parent_connection
        base_height = drive.origin.to_np()[2, 3]
        ranked = sorted(
            (
                (turn, distance, offset)
                for turn in candidates.turns
                for distance in candidates.distances
                for offset in candidates.offsets_to_the_left
            ),
            key=lambda candidate: (
                candidates.turns.index(candidate[0])
                + candidates.distances.index(candidate[1])
                + candidates.offsets_to_the_left.index(candidate[2])
            ),
        )
        stands = []
        with self.world.reset_state_context():
            for turn, distance, offset in ranked:
                angle = base_angle + turn
                front = np.array([math.cos(angle), math.sin(angle)])
                left_of_the_robot = np.array([front[1], -front[0]])
                position = (
                    np.asarray(point[:2])
                    + distance * front
                    + offset * left_of_the_robot
                )
                world_T_stand = HomogeneousTransformationMatrix.from_xyz_rpy(
                    position[0],
                    position[1],
                    base_height,
                    yaw=angle + math.pi,
                    reference_frame=self.world.root,
                )
                drive.origin = world_T_stand
                if self.clearance.keeps_clear(arm_margin=self.arm_margin):
                    stands.append(world_T_stand.pose)
                if len(stands) >= candidates.number:
                    break
        logger.info("%d stands facing %s", len(stands), np.round(point, 3))
        return stands

    def stand_clear_of_the_start(
        self, step: float = 0.05, maximum_distance: float = 1.0
    ) -> bool:
        """
        Step the robot back from RoboCasa's start pose, and sideways where stepping back
        alone does not do, until it keeps clear of the kitchen; the stands nearer the
        start pose, and straighter behind it, are tried first.

        :param step: How far the robot steps at a time.
        :param maximum_distance: How far back and to either side of the start pose the
            robot stands at most.
        :return: Whether a stand clear of the kitchen was found; the robot stands at the
            last one tried if not.
        """
        start = self.start_pose.to_np()
        drive = self.robot.root.parent_connection
        offsets = sorted(
            (
                (back, side)
                for back in np.arange(0.0, maximum_distance + step / 2, step)
                for side in np.arange(
                    -maximum_distance / 2, maximum_distance / 2 + step / 2, step
                )
            ),
            key=lambda offset: (abs(offset[0]) + 2 * abs(offset[1]), abs(offset[1])),
        )
        for back, side in offsets:
            start_T_stand = np.eye(4)
            start_T_stand[0, 3] = -back
            start_T_stand[1, 3] = side
            drive.origin = start @ start_T_stand
            if self.clearance.keeps_clear():
                return True
        return False

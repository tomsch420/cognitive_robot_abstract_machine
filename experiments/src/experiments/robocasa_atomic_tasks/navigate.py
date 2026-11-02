"""
A PR2 performs RoboCasa's navigation task (``kitchen_navigate.py``): it drives along a
path through the kitchen's free space to stand in front of the fixture RoboCasa names.

RoboCasa says where its own robot would stand, and accepts a base up to 0.2 m from
there that faces the fixture. The PR2 is wider than RoboCasa's robot, so where that
spot lies too close to the kitchen for the PR2, it stands at the nearest spot within
RoboCasa's tolerance that it fits.

Run it with::

    python -m experiments.robocasa_atomic_tasks.navigate NavigateKitchen
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np
from typing_extensions import ClassVar

from coraplex.robot_plans.actions.core.robot_body import ParkArmsAction
from coraplex.plans.failures import MotionMadeNoProgress
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Pose,
)

from experiments.robocasa_atomic_tasks.episode import (
    RoboCasaAtomicTask,
    RoboCasaEpisode,
    main,
)
from experiments.robocasa_atomic_tasks.skills import StandAction

logger = logging.getLogger(__name__)


@dataclass
class NavigateKitchen(RoboCasaAtomicTask):
    """
    Drive to where RoboCasa would stand its own robot in front of the target fixture, or
    as near to it as the PR2 fits, facing the fixture.
    """

    tolerance: ClassVar[float] = 0.18
    """
    How far from RoboCasa's spot the PR2 may stand, in metres; RoboCasa accepts 0.2.
    """

    goal_search_radius: ClassVar[float] = 0.8
    """
    How far from its final spot the path through the free space may end, in metres; the
    PR2 drives the rest of the way straight.
    """

    start_search_radius: ClassVar[float] = 0.5
    """
    How far from where it starts the PR2 looks for a spot in the free space to set off
    from, in metres: it starts close to a counter, nearer than the free space it drives
    through keeps clear of the kitchen.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        world = episode.world
        base_pose = episode.robot.root.global_pose.to_np()
        start_yaw = math.atan2(base_pose[1, 0], base_pose[0, 0])
        goal = self.nearest_clear_spot(
            episode, np.array(environment.target_pos[:2]), environment.target_ori[2]
        )
        start, path_end, waypoints = self.widest_path(episode, base_pose[:2, 3], goal)
        steps = [
            ParkArmsAction(arms=episode.robot.all_arms),
            StandAction(target_location=self.base_pose(episode, start, start_yaw)),
        ]
        # Turned while driving, the arms the PR2 parks beside it sweep through what it
        # passes; it turns on the spot at every waypoint and drives straight between.
        for waypoint, next_waypoint in zip(waypoints[:-1], waypoints[1:]):
            here = self.in_the_root_frame(episode, waypoint)
            there = self.in_the_root_frame(episode, next_waypoint)
            if np.linalg.norm(there - here) < 1e-3:
                continue
            yaw = math.atan2(*(there - here)[::-1])
            steps.append(
                StandAction(target_location=self.base_pose(episode, here, yaw))
            )
            steps.append(
                StandAction(target_location=self.base_pose(episode, there, yaw))
            )
        steps.append(
            StandAction(
                target_location=self.base_pose(
                    episode, path_end, environment.target_ori[2]
                )
            )
        )
        steps.append(
            StandAction(
                target_location=self.base_pose(episode, goal, environment.target_ori[2])
            )
        )
        episode.perform(steps[0])
        # A base driven by its wheels' velocities in a physical simulation may stall
        # short of a waypoint near the kitchen, held off by collision avoidance; it
        # drives on to the next one from there, and RoboCasa's tolerance judges where
        # it ends up.
        for step in steps[1:]:
            try:
                episode.perform(step)
            except MotionMadeNoProgress as stall:
                logger.warning(
                    "Driving on after the base stalled: %s",
                    str(stall).splitlines()[0][:200],
                )

    def widest_path(
        self, episode: RoboCasaEpisode, start: np.ndarray, goal: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, list]:
        """
        :param episode: The episode the task is performed in.
        :param start: Where the PR2 stands.
        :param goal: Where it is to stand.
        :return: Where in the free space the path starts and ends, near ``start`` and
            ``goal``, and its waypoints, through the free space kept clearest of the
            kitchen that still has a path: as far as the PR2's parked arms reach out
            at best, as far as its base does at worst.
        :raises PointOccupiedError: If not even the free space kept clear by the base
            alone has a path.
        """
        world = episode.world
        robot = episode.robot
        footprint = robot.as_bounding_box_collection_in_frame(robot.root).bounding_box()
        widest = max(
            abs(footprint.min_x), footprint.max_x, abs(footprint.min_y), footprint.max_y
        )
        base_radius = robot.mobile_base.base_radius
        failure = None
        for bloat in np.linspace(widest, base_radius, 5):
            free_space = self.free_space(episode, bloat)
            path_start = self.nearest_free_spot(
                episode, free_space, start, self.start_search_radius
            )
            path_end = self.nearest_free_spot(
                episode, free_space, goal, self.goal_search_radius
            )
            try:
                waypoints = free_space.path_from_to(
                    Point2(*path_start, reference_frame=world.root),
                    Point2(*path_end, reference_frame=world.root),
                )
            except Exception as error:
                failure = error
                continue
            if waypoints:
                logger.info("Path kept %.2f m clear of the kitchen", bloat)
                return path_start, path_end, waypoints
        raise failure

    @staticmethod
    def in_the_root_frame(episode: RoboCasaEpisode, point: Point2) -> np.ndarray:
        """
        :param episode: The episode the task is performed in.
        :param point: A point on the floor.
        :return: Its coordinates in the world's root frame.
        """
        world = episode.world
        in_root = world.transform(point.to_point3(), world.root)
        return np.array([float(in_root.x), float(in_root.y)])

    def nearest_clear_spot(
        self, episode: RoboCasaEpisode, spot: np.ndarray, yaw: float
    ) -> np.ndarray:
        """
        :param episode: The episode the task is performed in.
        :param spot: Where RoboCasa would stand its robot.
        :param yaw: Which way the robot is to face there.
        :return: The spot nearest to it within :attr:`tolerance` where the PR2, facing
            that way, keeps clear of the kitchen; the spot itself if there is none.
        """
        drive = episode.robot.root.parent_connection
        original = drive.origin.to_np()
        try:
            for distance in np.arange(0.0, self.tolerance + 1e-9, 0.02):
                for angle in np.linspace(0.0, 2 * math.pi, 24, endpoint=False):
                    candidate = spot + distance * np.array(
                        [math.cos(angle), math.sin(angle)]
                    )
                    drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
                        candidate[0],
                        candidate[1],
                        original[2, 3],
                        yaw=yaw,
                        reference_frame=episode.world.root,
                    )
                    if episode.robot_keeps_clear():
                        return candidate
                    if distance == 0.0:
                        break
        finally:
            drive.origin = original
        return spot

    @staticmethod
    def base_pose(episode: RoboCasaEpisode, spot: np.ndarray, yaw: float) -> Pose:
        """
        :param episode: The episode the task is performed in.
        :param spot: Where on the floor the base stands.
        :param yaw: Which way it faces.
        :return: The pose of the base there, at the height it stands at now.
        """
        return HomogeneousTransformationMatrix.from_xyz_rpy(
            spot[0],
            spot[1],
            float(episode.robot.root.global_pose.to_np()[2, 3]),
            yaw=yaw,
            reference_frame=episode.world.root,
        ).pose

    def free_space(self, episode: RoboCasaEpisode, clearance: float):
        """
        :param episode: The episode the task is performed in.
        :param clearance: How far the free space keeps the middle of the base from the
            kitchen, in metres.
        :return: The free space of the floor the PR2 drives through.
        """
        robot = episode.robot
        return self.floor_under_the_robot(episode).planar_free_space(
            max_height=robot.as_bounding_box_collection_in_frame(robot.root)
            .bounding_box()
            .scale.z,
            bloat_obstacles=clearance,
        )

    @staticmethod
    def nearest_free_spot(
        episode: RoboCasaEpisode, free_space, spot: np.ndarray, radius: float
    ) -> np.ndarray:
        """
        :param episode: The episode the task is performed in.
        :param free_space: The free space the PR2 drives through.
        :param spot: A spot on the floor.
        :param radius: How far from it the PR2 may stand instead, in metres.
        :return: The spot itself if it lies in the free space, or else the nearest one
            within ``radius`` that does.
        """
        world = episode.world
        for distance in np.arange(0.0, radius + 1e-9, 0.02):
            for angle in np.linspace(0.0, 2 * math.pi, 24, endpoint=False):
                candidate = spot + distance * np.array(
                    [math.cos(angle), math.sin(angle)]
                )
                point = Point2(*candidate, reference_frame=world.root)
                if free_space._node_of_point(point) is not None:
                    return candidate
                if distance == 0.0:
                    break
        return spot

    @staticmethod
    def floor_under_the_robot(episode: RoboCasaEpisode) -> Floor:
        """
        :param episode: The episode the task is performed in.
        :return: The floor the robot stands on.
        """
        base = episode.robot.root.global_pose.to_np()[:3, 3]
        for floor in episode.world.get_semantic_annotations_by_type(Floor):
            box = floor.as_bounding_box_collection_at_origin(
                HomogeneousTransformationMatrix(reference_frame=episode.world.root)
            ).bounding_box()
            if box.min_x <= base[0] <= box.max_x and box.min_y <= base[1] <= box.max_y:
                return floor
        return episode.world.get_semantic_annotations_by_type(Floor)[0]


TASKS = [NavigateKitchen]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

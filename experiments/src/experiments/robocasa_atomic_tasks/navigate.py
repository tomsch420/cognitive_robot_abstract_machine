"""
A PR2 performs RoboCasa's navigation task (``kitchen_navigate.py``): it drives along a
path through the kitchen's free space to stand in front of the fixture RoboCasa names.

RoboCasa says where its own robot would stand, and accepts a base up to 0.2 m from there
that faces the fixture. Where that spot lies too close to the kitchen for the PR2, it
stands at the nearest spot within RoboCasa's tolerance that it fits.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

import numpy as np

from coraplex.plans.failures import MotionMadeNoProgress
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.exceptions import NoPathThroughTheKitchen
from experiments.robocasa_atomic_tasks.skills import StandAction
from experiments.robocasa_atomic_tasks.tasks import RoboCasaAtomicTask
from semantic_digital_twin.semantic_annotations.semantic_annotations import Floor
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
    Point2,
    Pose,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    PlanarGraphOfBoundingBoxes,
)

logger = logging.getLogger(__name__)


# %% a path across the floor


@dataclass
class FloorPath:
    """
    A path through the free space of the floor.
    """

    start: np.ndarray
    """
    Where in the free space the path starts, in the world's root frame.
    """

    end: np.ndarray
    """
    Where in the free space it ends.
    """

    waypoints: list[Point2]
    """
    The points the path runs through, the start first and the end last.
    """


# %% the task


@dataclass
class NavigateKitchen(RoboCasaAtomicTask):
    """
    Drive to where RoboCasa would stand its own robot in front of the target fixture, or
    as near to it as the PR2 fits, facing the fixture.

    Turned while driving, the arms the PR2 parks beside it sweep through what it passes;
    it turns on the spot at every waypoint and drives straight between. A base that
    stalls short of a waypoint drives on to the next one from there.
    """

    tolerance: float = 0.18
    """
    How far from RoboCasa's spot the PR2 may stand, in metres; RoboCasa accepts 0.2.
    """

    goal_search_radius: float = 0.8
    """
    How far from its final spot the path through the free space may end, in metres; the
    PR2 drives the rest of the way straight.
    """

    start_search_radius: float = 0.5
    """
    How far from where it starts the PR2 looks for a spot in the free space to set off
    from, in metres.
    """

    search_step: float = 0.02
    """
    How far apart the rings of spots searched lie, in metres.
    """

    spots_per_ring: int = 24
    """
    How many spots are searched on each ring.
    """

    clearances_tried: int = 5
    """
    How many clearances from the kitchen, from what the PR2's parked arms need down to
    what its base needs, a path is searched for with.
    """

    minimum_drive: float = 1e-3
    """
    How short a leg between two waypoints is skipped, in metres.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        base_pose = episode.robot.root.global_pose.to_np()
        start_yaw = math.atan2(base_pose[1, 0], base_pose[0, 0])
        target_yaw = environment.target_ori[2]
        goal = self.nearest_clear_spot(
            episode, np.array(environment.target_pos[:2]), target_yaw
        )
        path = self.widest_path(episode, base_pose[:2, 3], goal)
        episode.perform(lambda: [episode.create_parking_step()]).raise_if_failed()
        for target in self.stands_along(episode, path, start_yaw, goal, target_yaw):
            outcome = episode.perform(lambda: [StandAction(target_location=target)])
            if isinstance(outcome.failure, MotionMadeNoProgress):
                logger.warning(
                    "Driving on after the base stalled: %s",
                    str(outcome.failure).splitlines()[0][:200],
                )
                continue
            outcome.raise_if_failed()

    def stands_along(
        self,
        episode: RoboCasaEpisode,
        path: FloorPath,
        start_yaw: float,
        goal: np.ndarray,
        goal_yaw: float,
    ) -> list[Pose]:
        """
        :param episode: The episode the task is performed in.
        :param path: The path through the free space.
        :param start_yaw: Which way the PR2 faces where it starts.
        :param goal: Where it is to stand at last.
        :param goal_yaw: Which way it is to face there.
        :return: The poses the base is driven to one after another: the path's start,
            each waypoint turned towards and reached, the path's end, and the goal.
        """
        stands = [self.base_pose(episode, path.start, start_yaw)]
        for waypoint, next_waypoint in zip(path.waypoints[:-1], path.waypoints[1:]):
            here = self.in_the_root_frame(episode, waypoint)
            there = self.in_the_root_frame(episode, next_waypoint)
            if np.linalg.norm(there - here) < self.minimum_drive:
                continue
            yaw = math.atan2(*(there - here)[::-1])
            stands.append(self.base_pose(episode, here, yaw))
            stands.append(self.base_pose(episode, there, yaw))
        stands.append(self.base_pose(episode, path.end, goal_yaw))
        stands.append(self.base_pose(episode, goal, goal_yaw))
        return stands

    def widest_path(
        self, episode: RoboCasaEpisode, start: np.ndarray, goal: np.ndarray
    ) -> FloorPath:
        """
        :param episode: The episode the task is performed in.
        :param start: Where the PR2 stands.
        :param goal: Where it is to stand.
        :return: The path through the free space kept clearest of the kitchen that
            still has one: as far as the PR2's parked arms reach out at best, as far as
            its base does at worst.
        :raises NoPathThroughTheKitchen: If not even the free space kept clear by the
            base alone has a path.
        """
        world = episode.world
        robot = episode.robot
        footprint = robot.as_bounding_box_collection_in_frame(robot.root).bounding_box()
        widest = max(
            abs(footprint.min_x), footprint.max_x, abs(footprint.min_y), footprint.max_y
        )
        base_radius = robot.mobile_base.base_radius
        for clearance in np.linspace(widest, base_radius, self.clearances_tried):
            free_space = self.free_space(episode, clearance)
            path_start = self.nearest_free_spot(
                episode, free_space, start, self.start_search_radius
            )
            path_end = self.nearest_free_spot(
                episode, free_space, goal, self.goal_search_radius
            )
            if path_start is None or path_end is None:
                continue
            waypoints = free_space.path_from_to(
                Point2(*path_start, reference_frame=world.root),
                Point2(*path_end, reference_frame=world.root),
            )
            if waypoints:
                logger.info("Path kept %.2f m clear of the kitchen", clearance)
                return FloorPath(start=path_start, end=path_end, waypoints=waypoints)
        raise NoPathThroughTheKitchen(smallest_clearance=base_radius)

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

    def spots_around(self, spot: np.ndarray, radius: float) -> list[np.ndarray]:
        """
        :param spot: A spot on the floor.
        :param radius: How far from it the spots lie at most, in metres.
        :return: The spot itself, then the spots on rings around it, nearest first.
        """
        return [spot] + [
            spot + distance * np.array([math.cos(angle), math.sin(angle)])
            for distance in np.arange(
                self.search_step, radius + self.search_step / 2, self.search_step
            )
            for angle in np.linspace(
                0.0, 2 * math.pi, self.spots_per_ring, endpoint=False
            )
        ]

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
        height = drive.origin.to_np()[2, 3]
        with episode.world.reset_state_context():
            for candidate in self.spots_around(spot, self.tolerance):
                drive.origin = HomogeneousTransformationMatrix.from_xyz_rpy(
                    candidate[0],
                    candidate[1],
                    height,
                    yaw=yaw,
                    reference_frame=episode.world.root,
                )
                if episode.clearance.keeps_clear():
                    return candidate
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

    @staticmethod
    def free_space(
        episode: RoboCasaEpisode, clearance: float
    ) -> PlanarGraphOfBoundingBoxes:
        """
        :param episode: The episode the task is performed in.
        :param clearance: How far the free space keeps the middle of the base from the
            kitchen, in metres.
        :return: The free space of the floor under the PR2.
        """
        robot = episode.robot
        return NavigateKitchen.floor_under_the_robot(episode).planar_free_space(
            max_height=robot.as_bounding_box_collection_in_frame(robot.root)
            .bounding_box()
            .scale.z,
            bloat_obstacles=clearance,
        )

    def nearest_free_spot(
        self,
        episode: RoboCasaEpisode,
        free_space: PlanarGraphOfBoundingBoxes,
        spot: np.ndarray,
        radius: float,
    ) -> np.ndarray | None:
        """
        :param episode: The episode the task is performed in.
        :param free_space: The free space the PR2 drives through.
        :param spot: A spot on the floor.
        :param radius: How far from it the PR2 may stand instead, in metres.
        :return: The spot itself if it lies in the free space, or else the nearest one
            within ``radius`` that does; ``None`` if none does.
        """
        root = episode.world.root
        return next(
            (
                candidate
                for candidate in self.spots_around(spot, radius)
                if free_space._node_of_point(Point2(*candidate, reference_frame=root))
                is not None
            ),
            None,
        )

    @staticmethod
    def floor_under_the_robot(episode: RoboCasaEpisode) -> Floor:
        """
        :param episode: The episode the task is performed in.
        :return: The floor the robot stands on; the first floor if it stands on none.
        """
        base = episode.robot.root.global_pose.to_np()[:3, 3]
        floors = episode.world.get_semantic_annotations_by_type(Floor)
        for floor in floors:
            box = floor.as_bounding_box_collection_at_origin(
                HomogeneousTransformationMatrix(reference_frame=episode.world.root)
            ).bounding_box()
            if box.min_x <= base[0] <= box.max_x and box.min_y <= base[1] <= box.max_y:
                return floor
        return floors[0]

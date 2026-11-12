"""
Paths for a point such as the gripper's tool frame through the free space of the
kitchen, planned on a graph of convex sets: the boxes the space around the way
decomposes into once the boxes bounding everything but the robot, bloated by how far the
path keeps from them, are taken out of it.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

import numpy as np

from semantic_digital_twin.robots.robot_parts import AbstractRobot
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    CoordinateBounds,
    VolumetricBoundingBox,
)
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    VolumetricGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


# %% a point of the free space


@dataclass
class NearestFreePoint:
    """
    The point of the free space nearest to another point.
    """

    point: np.ndarray
    """
    The point of the free space, a little inside the box it lies in.
    """

    distance: float
    """
    How far it is from the other point.
    """


# %% planning a path


@dataclass
class FreeSpacePathPlanner:
    """
    Plans paths for a point through the free space around the robot.
    """

    world: World
    """
    The world the path runs through.
    """

    robot: AbstractRobot
    """
    The robot, whose bodies, and what it holds, are no obstacles.
    """

    clearance: float = 0.05
    """
    How far the path keeps from everything else, in metres.
    """

    margin: float = 0.35
    """
    How far beyond the box spanned by the start and the goal the space searched reaches,
    in metres.
    """

    floor_height: float = 0.1
    """
    How high above the floor the path keeps at least, in metres.
    """

    maximum_snap: float = 0.12
    """
    How far from the start or the goal the free space may begin, if either lies within
    the clearance of something, for the path to leave or enter it straight from there,
    in metres.
    """

    box_inset: float = 0.005
    """
    How far inside its box a point of the free space is taken, in metres.
    """

    ignored: list[Body] = field(default_factory=list)
    """
    Bodies that are no obstacles, such as a part to be taken hold of.
    """

    def path(self, start: np.ndarray, goal: np.ndarray) -> list[np.ndarray] | None:
        """
        :param start: Where the path starts, in the world's root frame.
        :param goal: Where it ends.
        :return: The points of the path, the start first and the goal last, or ``None``
            if the free space connects no points near the two.
        """
        began = time.time()
        searched = self.searched_space(start, goal)
        root = self.world.root
        search_space = BoundingBoxCollection(
            [
                VolumetricBoundingBox(
                    *searched.lower,
                    *searched.upper,
                    HomogeneousTransformationMatrix(reference_frame=root),
                )
            ],
            reference_frame=root,
        )
        obstacles = BoundingBoxCollection(self.obstacle_boxes(searched), root)
        graph = VolumetricGraphOfBoundingBoxes(
            world=self.world, search_space=search_space
        )
        for box in BoundingBoxCollection.from_event(
            VolumetricBoundingBox,
            reference_frame=root,
            event=VolumetricGraphOfBoundingBoxes.free_space_from_bounding_boxes(
                obstacles, search_space.event
            ),
        ):
            graph.add_node(box)
        if not graph.graph.nodes():
            return None
        graph.calculate_connectivity()
        entry = self.nearest_free_point(graph, np.asarray(start, dtype=float))
        exit_ = self.nearest_free_point(graph, np.asarray(goal, dtype=float))
        if max(entry.distance, exit_.distance) > self.maximum_snap:
            logger.info(
                "No free space within %.2f m of the start (%.3f m) or the goal (%.3f m)",
                self.maximum_snap,
                entry.distance,
                exit_.distance,
            )
            return None
        path = graph.path_from_to(
            Point3(*entry.point, reference_frame=root),
            Point3(*exit_.point, reference_frame=root),
        )
        logger.info(
            "Free space of %d boxes around %d obstacles, path of %s points, in %.1f s",
            len(graph.graph.nodes()),
            len(obstacles),
            None if path is None else len(path),
            time.time() - began,
        )
        if path is None:
            return None
        points = [np.asarray(point.to_np()[:3], dtype=float) for point in path]
        if entry.distance > 0.0:
            points.insert(0, np.asarray(start, dtype=float))
        if exit_.distance > 0.0:
            points.append(np.asarray(goal, dtype=float))
        return points

    def searched_space(self, start: np.ndarray, goal: np.ndarray) -> CoordinateBounds:
        """
        :param start: Where the path starts, in the world's root frame.
        :param goal: Where it ends.
        :return: The space searched for the path, :attr:`margin` around the two and
            above :attr:`floor_height`.
        """
        lower = np.minimum(start, goal) - self.margin
        upper = np.maximum(start, goal) + self.margin
        lower[2] = max(lower[2], self.floor_height)
        return CoordinateBounds(lower=lower, upper=upper)

    def obstacle_boxes(self, searched: CoordinateBounds) -> list[VolumetricBoundingBox]:
        """
        :param searched: The space searched, in the world's root frame.
        :return: The boxes bounding every shape of every body with collision geometry
            but the robot's and :attr:`ignored`, bloated by :attr:`clearance`, that
            reach into the space searched.
        """
        skipped = set(
            self.world.get_kinematic_structure_entities_of_branch(self.robot.root)
        ) | set(self.ignored)
        boxes = []
        for body in self.world.bodies_with_collision:
            if body in skipped:
                continue
            for box in body.collision.as_bounding_box_collection_in_frame(
                self.world.root
            ):
                box = box.bloat(self.clearance, self.clearance, self.clearance)
                corners = box.to_array_bounds()
                if np.all(corners.upper > searched.lower) and np.all(
                    corners.lower < searched.upper
                ):
                    boxes.append(box)
        return boxes

    def nearest_free_point(
        self, graph: VolumetricGraphOfBoundingBoxes, point: np.ndarray
    ) -> NearestFreePoint:
        """
        :param graph: A graph of the free space.
        :param point: A point, in the graph's reference frame.
        :return: The point of the free space nearest to it.
        """
        nearest = NearestFreePoint(point=point, distance=np.inf)
        for box in graph.graph.nodes():
            corners = box.to_array_bounds()
            inset = np.minimum(self.box_inset, (corners.upper - corners.lower) / 2)
            candidate = np.clip(point, corners.lower + inset, corners.upper - inset)
            distance = float(np.linalg.norm(candidate - point))
            if distance < nearest.distance:
                nearest = NearestFreePoint(point=candidate, distance=distance)
        return nearest

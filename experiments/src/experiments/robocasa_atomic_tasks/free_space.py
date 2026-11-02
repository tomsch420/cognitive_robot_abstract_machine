"""
Paths for the gripper through the free space of the kitchen, planned on a graph of
convex sets: the boxes the space around the gripper's way decomposes into once the boxes
bounding everything but the robot, bloated by how far the gripper keeps from them, are
taken out of it.

Collision avoidance alone pulls the gripper straight towards its goal and stops it
wherever the kitchen stands in the way, in front of a cabinet's floor or beside the door
of a microwave. A path around those leads it to where the straight way is free.
"""

from __future__ import annotations

import logging
import time

import numpy as np
from typing_extensions import List, Sequence

from semantic_digital_twin.robots.robot_parts import AbstractRobot
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix, Point3
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import VolumetricBoundingBox
from semantic_digital_twin.world_description.graph_of_convex_sets.boxes import (
    VolumetricGraphOfBoundingBoxes,
)
from semantic_digital_twin.world_description.shape_collection import (
    BoundingBoxCollection,
)
from semantic_digital_twin.world_description.world_entity import Body

logger = logging.getLogger(__name__)


def obstacle_boxes(
    world: World,
    robot: AbstractRobot,
    lower: np.ndarray,
    upper: np.ndarray,
    clearance: float,
    ignored: Sequence[Body] = (),
) -> List[VolumetricBoundingBox]:
    """
    :param world: The world.
    :param robot: The robot, whose bodies, and what it holds, are no obstacles.
    :param lower: The lower corner of the space searched, in the world's root frame.
    :param upper: Its upper corner.
    :param clearance: By how much every box is bloated on every side, in metres.
    :param ignored: Further bodies that are no obstacles.
    :return: The bloated boxes bounding every shape of every other body with collision
        geometry that reach into the space searched.
    """
    skipped = set(world.get_kinematic_structure_entities_of_branch(robot.root)) | set(
        ignored
    )
    boxes = []
    for body in world.bodies_with_collision:
        if body in skipped:
            continue
        for box in body.collision.as_bounding_box_collection_in_frame(world.root):
            box = box.bloat(clearance, clearance, clearance)
            if (
                box.max_x > lower[0]
                and box.min_x < upper[0]
                and box.max_y > lower[1]
                and box.min_y < upper[1]
                and box.max_z > lower[2]
                and box.min_z < upper[2]
            ):
                boxes.append(box)
    return boxes


def corners_of(box: VolumetricBoundingBox) -> tuple[np.ndarray, np.ndarray]:
    """
    :param box: A box, axis-aligned in the world's root frame.
    :return: Its lower and upper corner there; a box of free space holds its extents
        about its own origin.
    """
    offset = box.origin.to_np()[:3, 3]
    return (
        offset + np.array([box.min_x, box.min_y, box.min_z]),
        offset + np.array([box.max_x, box.max_y, box.max_z]),
    )


def nearest_free_point(
    graph: VolumetricGraphOfBoundingBoxes, point: np.ndarray
) -> tuple[np.ndarray, float]:
    """
    :param graph: A graph of the free space.
    :param point: A point, in the graph's reference frame.
    :return: The point of the free space nearest to it, a little inside the box it lies
        in, and how far that is from the point.
    """
    best, best_distance = point, np.inf
    for box in graph.graph.nodes():
        box_lower, box_upper = corners_of(box)
        inset = np.minimum(0.005, (box_upper - box_lower) / 2)
        candidate = np.clip(point, box_lower + inset, box_upper - inset)
        distance = float(np.linalg.norm(candidate - point))
        if distance < best_distance:
            best, best_distance = candidate, distance
    return best, best_distance


def path_through_free_space(
    world: World,
    robot: AbstractRobot,
    start: np.ndarray,
    goal: np.ndarray,
    clearance: float = 0.05,
    margin: float = 0.35,
    floor_height: float = 0.1,
    maximum_snap: float = 0.12,
    ignored: Sequence[Body] = (),
) -> List[np.ndarray] | None:
    """
    Plan a path for a point, such as the robot's tool frame, from one point to another
    through the free space around the two.

    :param world: The world.
    :param robot: The robot, whose bodies are no obstacles.
    :param start: Where the path starts, in the world's root frame.
    :param goal: Where it ends.
    :param clearance: How far the path keeps from everything else, in metres.
    :param margin: How far beyond the box spanned by the start and the goal the space
        searched reaches, in metres.
    :param floor_height: How high above the floor the path keeps at least, in metres.
    :param maximum_snap: How far from the start or the goal the free space may begin, if
        either lies within the clearance of something, for the path to leave or enter it
        straight from there, in metres.
    :param ignored: Bodies that are no obstacles, such as a part to be taken hold of.
    :return: The points of the path, the start first and the goal last, or ``None`` if
        the free space connects no points near the two.
    """
    began = time.time()
    lower = np.minimum(start, goal) - margin
    upper = np.maximum(start, goal) + margin
    lower[2] = max(lower[2], floor_height)
    root = world.root
    search_space = BoundingBoxCollection(
        [
            VolumetricBoundingBox(
                *lower, *upper, HomogeneousTransformationMatrix(reference_frame=root)
            )
        ],
        reference_frame=root,
    )
    obstacles = BoundingBoxCollection(
        obstacle_boxes(world, robot, lower, upper, clearance, ignored), root
    )
    graph = VolumetricGraphOfBoundingBoxes(world=world, search_space=search_space)
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
    entry, entry_distance = nearest_free_point(graph, np.asarray(start, dtype=float))
    exit_, exit_distance = nearest_free_point(graph, np.asarray(goal, dtype=float))
    if max(entry_distance, exit_distance) > maximum_snap:
        logger.info(
            "No free space within %.2f m of the start (%.3f m) or the goal (%.3f m)",
            maximum_snap,
            entry_distance,
            exit_distance,
        )
        return None
    path = graph.path_from_to(
        Point3(*entry, reference_frame=root), Point3(*exit_, reference_frame=root)
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
    if entry_distance > 1e-6:
        points.insert(0, np.asarray(start, dtype=float))
    if exit_distance > 1e-6:
        points.append(np.asarray(goal, dtype=float))
    return points

"""
Tests of the paths a point takes through the free space around the robot.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.free_space import FreeSpacePathPlanner
from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.specifications.robots import RobotSpecification
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Scale
from semantic_digital_twin.world_description.world_entity import Body

from .conftest import box_body

# %% paths through open space


def test_a_path_through_open_space_away_from_the_origin_joins_its_start_and_goal(
    panda_omron,
):
    """
    Nothing but the robot stands around a start and a goal far from the world's origin,
    so the path between them begins at the start and ends at the goal.
    """
    start = np.array([2.0, 0.0, 1.0])
    goal = np.array([2.5, 0.3, 1.2])
    planner = FreeSpacePathPlanner(world=panda_omron._world, robot=panda_omron)

    path = planner.path(start, goal)

    assert path is not None
    np.testing.assert_allclose(path[0], start)
    np.testing.assert_allclose(path[-1], goal)


# %% room for an arm above the point


@dataclass
class RobotBesideAShelf:
    """
    The PandaOmron in a kitchen of nothing but a shelf hanging free far from the robot.
    """

    robot: PandaOmron
    """
    The robot, whose world holds the shelf.
    """

    shelf_center: np.ndarray
    """
    Where the shelf's centre lies, in the world's root frame.
    """

    shelf_scale: Scale
    """
    The shelf's extents.
    """


@pytest.fixture
def robot_beside_a_shelf() -> RobotBesideAShelf:
    """
    A shelf 1 m wide and 5 cm thick whose underside lies 22.5 cm above the height of a
    straight way under it.
    """
    world = World()
    shelf_center = np.array([2.3, 0.0, 1.25])
    shelf_scale = Scale(0.1, 1.0, 0.05)
    with world.modify_world():
        root = Body(name=PrefixedName("kitchen"))
        world.add_kinematic_structure_entity(root)
        box_body(world, root, "shelf", shelf_scale, shelf_center)
    robot = RobotSpecification(PandaOmron).spawn(world)
    return RobotBesideAShelf(
        robot=robot, shelf_center=shelf_center, shelf_scale=shelf_scale
    )


def points_along(path: list[np.ndarray], step: float = 0.01) -> list[np.ndarray]:
    """
    :param path: The points of a path.
    :param step: How far apart the points taken along each segment lie at most.
    :return: Points along every segment of the path, its ends included.
    """
    points = []
    for start, end in zip(path[:-1], path[1:]):
        count = max(2, int(np.ceil(np.linalg.norm(end - start) / step)) + 1)
        points.extend(
            start + (end - start) * share for share in np.linspace(0, 1, count)
        )
    return points


def test_a_path_keeps_the_room_below_an_obstacle_free(robot_beside_a_shelf):
    """
    The straight way from one side of the shelf to the other runs under it, within the
    room an arm above the point needs, so the path passes no point of that room.
    """
    room = 0.3
    world = robot_beside_a_shelf.robot._world
    planner = FreeSpacePathPlanner(
        world=world, robot=robot_beside_a_shelf.robot, room_below_obstacles=room
    )
    center = robot_beside_a_shelf.shelf_center
    half = (
        np.array(
            [
                robot_beside_a_shelf.shelf_scale.x,
                robot_beside_a_shelf.shelf_scale.y,
                robot_beside_a_shelf.shelf_scale.z,
            ]
        )
        / 2
    )
    start = np.array([2.0, 0.0, 1.0])
    goal = np.array([2.6, 0.0, 1.0])

    path = planner.path(start, goal)

    assert path is not None
    room_lower = center - half - np.array([0.0, 0.0, room])
    room_upper = center + half
    assert not any(
        np.all(point > room_lower) and np.all(point < room_upper)
        for point in points_along(path)
    )

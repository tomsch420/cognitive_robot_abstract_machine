"""
Tests of the paths a point takes through the free space around the robot.
"""

from __future__ import annotations

import numpy as np

from experiments.robocasa_atomic_tasks.free_space import FreeSpacePathPlanner

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

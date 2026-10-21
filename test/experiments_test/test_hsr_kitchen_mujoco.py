"""
The HSR picking a milk carton up from a table of the predefined kitchen, carrying it
along the table and setting it down there, simulated in MuJoCo with the carton held by
friction alone.
"""

import os

import mujoco
import pytest

from experiments.hsr_kitchen_mujoco.demo import (
    SceneBody,
    build_world,
    goal_pose,
    run,
    start_pose,
    table_extent,
)
from semantic_digital_twin.adapters.multi_sim import MujocoSim
from semantic_digital_twin.predefined_maps.kitchen_environment import (
    KitchenEnvironment,
)
from semantic_digital_twin.world import World

runs_in_ci = pytest.mark.skipif(
    os.environ.get("CI", "false").lower() == "false",
    reason="MuJoCo tests only run in CI",
)

MUJOCO_MISSING_ID = -1
"""
The id MuJoCo answers with for a name its model does not hold.
"""

PLACEMENT_TOLERANCE = 0.03
"""
How far, in metres, the milk may come to rest from where it was meant to be set down.
"""


@pytest.fixture(scope="module")
def world() -> World:
    return build_world()


@pytest.fixture(scope="module")
def world_after_the_run() -> World:
    world = build_world()
    run(world, headless=True)
    return world


def position_of(world: World, name: str):
    """
    :return: Where the world holds the body named ``name``.
    """
    return world.get_body_by_name(name).global_pose.position.to_np()[:3]


# %% the scene


def test_the_scene_is_the_predetermined_kitchen(world: World):
    kitchen = KitchenEnvironment().get_world()

    assert {body.name.name for body in kitchen.bodies} <= {
        body.name.name for body in world.bodies
    }


def test_the_milk_starts_on_the_table(world: World):
    extent = table_extent(world)
    x, y, _ = position_of(world, SceneBody.MILK)

    assert extent.min_x < x < extent.max_x
    assert extent.min_y < y < extent.max_y
    assert position_of(world, SceneBody.MILK) == pytest.approx(
        start_pose(world).position.to_np()[:3]
    )


@runs_in_ci
def test_every_body_of_the_scene_is_simulated(world: World):
    simulation = MujocoSim(world=world, headless=True)
    model = simulation.simulator._mj_model
    missing = [
        body.name.name
        for body in world.bodies
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body.name.name)
        == MUJOCO_MISSING_ID
    ]

    assert missing == []


# %% the run


@runs_in_ci
def test_the_hsr_sets_the_milk_down_where_it_was_meant_to(
    world_after_the_run: World,
):
    """
    Held by nothing but friction, the milk travels with the hand and comes to rest
    where the plan set it down.
    """
    assert position_of(world_after_the_run, SceneBody.MILK) == pytest.approx(
        goal_pose(world_after_the_run).position.to_np()[:3],
        abs=PLACEMENT_TOLERANCE,
    )


@runs_in_ci
def test_the_milk_is_never_attached_to_the_hand(world_after_the_run: World):
    """
    Picking the milk up does not move it under the hand in the world model, so it stays
    a body of the kitchen throughout.
    """
    milk = world_after_the_run.get_body_by_name(SceneBody.MILK)

    assert milk.parent_connection.parent is world_after_the_run.root

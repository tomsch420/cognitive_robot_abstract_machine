"""
Tests of trying grasps on the object of a pick and place task in its kitchen cut down to
the fixture the object lies in.
"""

from __future__ import annotations

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from ...pytest_environment import runs_in_continuous_integration
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.kitchen_grasp_trials import (
    EpisodeAroundTheObject,
    KitchenAroundTheObject,
    KitchenGraspTrials,
)
from experiments.robocasa_atomic_tasks.pick_place import PickPlaceSinkToCounter
from experiments.robocasa_atomic_tasks.robots import RobotModel
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.fixtures import Fridge, Stove

simulates_physics = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

# %% the kitchen cut down around the object in the sink


@pytest.fixture(scope="module")
def kitchen() -> KitchenAroundTheObject:
    return KitchenAroundTheObject(
        task=PickPlaceSinkToCounter(robot=RobotModel.PANDA_OMRON, physics=True)
    )


@pytest.fixture(scope="module")
def around_the_object(kitchen: KitchenAroundTheObject) -> EpisodeAroundTheObject:
    return kitchen.compose()


def test_the_object_lies_in_the_sink(around_the_object):
    assert around_the_object.fixture is around_the_object.episode.environment.sink


def test_the_sink_the_robot_and_the_object_stay(kitchen, around_the_object):
    episode = around_the_object.episode
    world_bodies = set(episode.world.kinematic_structure_entities)

    assert episode.fixture_body(episode.environment.sink) in world_bodies
    assert episode.object_body(kitchen.task.picked_object) in world_bodies
    assert episode.robot.root in world_bodies


@pytest.mark.parametrize("fixture_type", [Fridge, Stove])
def test_a_fixture_across_the_kitchen_is_removed(around_the_object, fixture_type):
    episode = around_the_object.episode
    [fixture] = [
        fixture
        for fixture in episode.environment.fixtures.values()
        if isinstance(fixture, fixture_type)
    ]
    names = {body.name.name for body in episode.world.bodies}

    assert fixture.root_body not in names


def test_no_semantic_annotation_refers_to_a_removed_body(around_the_object):
    world = around_the_object.episode.world
    entities = set(world.kinematic_structure_entities)

    assert all(annotation.root in entities for annotation in world.semantic_annotations)


def test_the_object_is_dropped_inside_the_sink(kitchen, around_the_object):
    episode = around_the_object.episode
    graspable = episode.graspable(kitchen.task.picked_object)
    sink_box = episode.bounds.of_branch(episode.fixture_body(around_the_object.fixture))

    area = around_the_object.inside_of_the_fixture(graspable)

    assert sink_box.lower[0] < area.x.lower < area.x.upper < sink_box.upper[0]
    assert sink_box.lower[1] < area.y.lower < area.y.upper < sink_box.upper[1]
    assert sink_box.lower[2] < area.height < graspable.root.global_pose.to_np()[2, 3]


# %% an attempt


@simulates_physics
def test_an_attempt_is_stored_with_where_the_object_lay(kitchen):
    database = GraspDatabase.connect("sqlite:///:memory:")
    trials = KitchenGraspTrials(
        kitchen=kitchen, database=database, number_of_attempts=1
    )

    [recorded] = trials.run()

    [stored] = database.attempts(recorded.attempt.task)
    assert stored.placement == recorded.attempt.placement
    assert stored.grasp.result.lifted == recorded.attempt.grasp.result.lifted

"""
Tests of trying grasps on the object of a pick and place task in its kitchen cut down to
the fixture the object lies in.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

pytest.importorskip("robosuite", reason="robosuite is not installed")

from ...pytest_environment import runs_in_continuous_integration
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.exceptions import NoRecordedGraspAttempt
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
    RecordedGraspAttempt,
)
from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.robocasa_atomic_tasks.judge import Judgement
from experiments.robocasa_atomic_tasks.stored_runs import (
    SimulatedMoment,
    StoredTaskRun,
)
from krrood_spatial_math.spatial_types import Twist, Vector3
from semantic_digital_twin.grasping.grasp_strategies import GraspFromAnySide
from semantic_digital_twin.grasping.object_relative_grasp import (
    GraspResult,
    draw_grasps,
)
from experiments.grasp_learning.kitchen_grasp_trials import (
    KitchenAroundTheObject,
    KitchenGraspTrials,
)
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
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
def episode(kitchen: KitchenAroundTheObject) -> RoboCasaEpisode:
    return kitchen.compose()


def test_the_sink_the_robot_and_the_object_stay(kitchen, episode):
    world_bodies = set(episode.world.kinematic_structure_entities)

    assert episode.fixture_body(episode.environment.sink) in world_bodies
    assert episode.object_body(kitchen.task.picked_object) in world_bodies
    assert episode.robot.root in world_bodies


@pytest.mark.parametrize("fixture_type", [Fridge, Stove])
def test_a_fixture_across_the_kitchen_is_removed(episode, fixture_type):
    [fixture] = [
        fixture
        for fixture in episode.environment.fixtures.values()
        if isinstance(fixture, fixture_type)
    ]
    names = {body.name.name for body in episode.world.bodies}

    assert fixture.root_body not in names


def test_no_semantic_annotation_refers_to_a_removed_body(episode):
    world = episode.world
    entities = set(world.kinematic_structure_entities)

    assert all(annotation.root in entities for annotation in world.semantic_annotations)


# %% storing an attempt with its run


def recorded_attempt(
    kitchen: KitchenAroundTheObject, episode: RoboCasaEpisode
) -> RecordedGraspAttempt:
    """
    :return: An attempt at a grasp drawn on the picked object, lifting it, with a run
        of two moments of one degree of freedom.
    """
    graspable = episode.graspable(kitchen.task.picked_object)
    [grasp] = draw_grasps(GraspFromAnySide().statement(graspable), 1)
    grasp.result = GraspResult(
        object_raised=True,
        motion_completed=True,
        object_displacement=Twist(translation=Vector3(0.0, 0.0, 0.2)),
        slip=Twist(translation=Vector3(0.0, 0.0, 0.0)),
    )
    return RecordedGraspAttempt(
        attempt=GraspAttempt(
            task=GraspLearningTask.of_graspable(
                graspable, type(episode.arm.end_effector)
            ),
            robot=type(episode.robot),
            object_name=graspable.root.name.name,
            source=GraspSource.STATED_REGIONS,
            placement=ObjectPlacement(x=1.0, y=-0.3),
            grasp=grasp,
        ),
        run=StoredTaskRun(
            task=kitchen.task.robocasa_task_name,
            layout=kitchen.task.layout,
            style=kitchen.task.style,
            seed=kitchen.task.seed,
            robot=kitchen.task.robot,
            degree_of_freedom_names=["lever"],
            moments=[
                SimulatedMoment(simulation_time=0.0, positions=[0.0]),
                SimulatedMoment(simulation_time=0.5, positions=[0.1]),
            ],
            judgement=Judgement(
                accomplished=False, judged_objects=[], judged_joint_positions=[]
            ),
        ),
    )


@pytest.fixture
def database() -> GraspDatabase:
    return GraspDatabase.connect("sqlite:///:memory:")


def test_a_recorded_attempt_counts_among_the_attempts_of_its_task(
    kitchen, episode, database
):
    recorded = recorded_attempt(kitchen, episode)

    database.add_attempts([recorded])

    [read] = database.attempts(recorded.attempt.task)
    assert read.grasp.height == recorded.attempt.grasp.height
    assert read.placement == recorded.attempt.placement


def test_a_recorded_attempt_is_read_back_with_its_run(kitchen, episode, database):
    recorded = recorded_attempt(kitchen, episode)
    database.add_attempts([recorded])

    read = database.recorded_attempt(1)

    assert read.run.moments == recorded.run.moments
    assert read.run.task == recorded.run.task


def test_reading_an_attempt_that_is_not_stored_raises(database):
    with pytest.raises(NoRecordedGraspAttempt):
        database.recorded_attempt(1)


# %% an attempt


@dataclass(kw_only=True)
class OneAttempt:
    """
    The trials of one attempt and what they stored.
    """

    trials: KitchenGraspTrials
    """
    The trials.
    """

    recorded: RecordedGraspAttempt
    """
    The attempt they tried.
    """


@pytest.fixture(scope="module")
def one_attempt(kitchen: KitchenAroundTheObject) -> OneAttempt:
    trials = KitchenGraspTrials(
        kitchen=kitchen,
        database=GraspDatabase.connect("sqlite:///:memory:"),
        number_of_attempts=1,
    )
    [recorded] = trials.run()
    return OneAttempt(trials=trials, recorded=recorded)


@simulates_physics
def test_an_attempt_is_stored_with_where_the_object_lay(one_attempt):
    recorded = one_attempt.recorded

    [stored] = one_attempt.trials.database.attempts(recorded.attempt.task)

    assert stored.placement == recorded.attempt.placement
    assert stored.grasp.result.lifted == recorded.attempt.grasp.result.lifted


@simulates_physics
def test_attempts_grasp_from_above(one_attempt):
    pitch = one_attempt.recorded.attempt.grasp.pitch

    assert pitch < one_attempt.trials.grasp_strategy.pitch.upper

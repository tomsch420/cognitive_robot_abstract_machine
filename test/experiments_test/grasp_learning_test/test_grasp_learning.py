"""
Storing grasp attempts, learning a grasp model from them, and handing it to an
annotation.
"""

from __future__ import annotations

from datetime import timedelta

import pytest
from typing_extensions import List

from ...pytest_environment import runs_in_continuous_integration

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.models import GraspModelLearner, GraspModelLibrary
from experiments.grasp_learning.pipeline import GraspLearningPipeline
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    GraspSource,
)
from experiments.physical_pick_up.objects import PickUpObject
from experiments.physical_pick_up.pick_up_experiment import PickUpExperiment
from experiments.physical_pick_up.robots import ObjectPlacement
from experiments.physical_pick_up.scene import PickUpScene
from semantic_digital_twin.spatial_types import Vector3
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Mug,
)
from semantic_digital_twin.grasping.surface_grasp import (
    GraspResult,
    draw_surface_grasps,
)

simulates_physics = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

# %% fixtures

LIFTING_HEIGHT = 0.5
"""
In the synthetic attempts, a grasp lifts the object exactly when it lies above this
fraction of the object's height.
"""


@pytest.fixture(scope="module")
def milk_scene() -> PickUpScene:
    return PickUpScene(object_description=PickUpObject.MILK.value)


@pytest.fixture
def database() -> GraspDatabase:
    return GraspDatabase.connect("sqlite:///:memory:")


def task_of(scene: PickUpScene) -> GraspLearningTask:
    """
    :return: The task of the scene's robot grasping the object in it.
    """
    return GraspLearningTask.of_graspable(scene.graspable, type(scene.arm.end_effector))


def synthetic_attempts(scene: PickUpScene, number: int) -> List[GraspAttempt]:
    """
    :return: Attempts at grasps drawn from the statement of the scene's object, which
        lift it exactly when they lie above :data:`LIFTING_HEIGHT`.
    """
    task = task_of(scene)
    grasps = draw_surface_grasps(
        scene.graspable.grasped_part().surface_grasp_statement(), number
    )
    for grasp in grasps:
        raised = grasp.height > LIFTING_HEIGHT
        grasp.result = GraspResult(
            object_raised=raised,
            motion_completed=True,
            object_displacement=Vector3(0.0, 0.0, 0.2 if raised else 0.0),
            object_rotation=Vector3(),
            translational_slip=Vector3(0.0, 0.0, 0.0 if raised else -0.2),
            rotational_slip=Vector3(),
        )
    return [
        GraspAttempt(
            task=task,
            robot=type(scene.robot),
            object_name=scene.graspable.root.name.name,
            source=GraspSource.STATED_REGIONS,
            placement=scene.pick_up_area.middle(),
            grasp=grasp,
        )
        for grasp in grasps
    ]


# %% tasks and attempts


def test_the_task_of_an_object_names_object_part_and_gripper(milk_scene):
    milk = milk_scene.graspable
    gripper = type(milk_scene.arm.end_effector)

    task = GraspLearningTask.of_graspable(milk, gripper)

    assert task == GraspLearningTask(
        annotation_type=type(milk),
        grasped_part=type(milk.grasped_part()),
        gripper=gripper,
    )


def test_stored_attempts_are_read_back_by_task(milk_scene, database):
    attempts = synthetic_attempts(milk_scene, 5)
    other_task = GraspLearningTask(
        annotation_type=Mug,
        grasped_part=Handle,
        gripper=type(milk_scene.arm.end_effector),
    )

    database.add_attempts(attempts)

    read = database.attempts(attempts[0].task)
    assert [attempt.grasp.height for attempt in read] == [
        attempt.grasp.height for attempt in attempts
    ]
    assert read[0].placement == attempts[0].placement
    assert result_values(read[0].grasp.result) == pytest.approx(
        result_values(attempts[0].grasp.result)
    )
    assert database.attempts(other_task) == []


def result_values(result: GraspResult) -> List[float]:
    """
    :return: Every value of ``result``, the components of its vectors one by one, since
        vectors compare symbolically.
    """
    vectors = (
        result.object_displacement,
        result.object_rotation,
        result.translational_slip,
        result.rotational_slip,
    )
    return [
        float(result.object_raised),
        float(result.motion_completed),
        *(float(value) for vector in vectors for value in vector.to_np()[:3]),
    ]


# %% learning and handing out models


def test_a_learned_model_asks_for_lifting_grasps_after_storage(milk_scene, database):
    attempts = synthetic_attempts(milk_scene, 300)
    task = attempts[0].task
    database.add_model(
        GraspModelLearner(minimum_attempts_per_leaf=20).learn(task, attempts)
    )

    [model] = database.model_library().models
    grasps = draw_surface_grasps(
        milk_scene.graspable.surface_grasp_statement(require_lifting=True),
        30,
        model.model_registry(),
    )

    assert model.task == task
    assert model.number_of_attempts == len(attempts)
    for grasp in grasps:
        assert grasp.height > LIFTING_HEIGHT


def test_the_library_hands_a_model_to_the_annotation_of_its_task(milk_scene):
    attempts = synthetic_attempts(milk_scene, 300)
    task = attempts[0].task
    library = GraspModelLibrary(
        models=[GraspModelLearner(minimum_attempts_per_leaf=20).learn(task, attempts)]
    )
    milk = milk_scene.graspable
    lowest, highest = milk.grasp_surface().bounds
    lifting_height = lowest[2] + LIFTING_HEIGHT * (highest[2] - lowest[2])

    grasps = library.grasp_candidates(milk, task.gripper)

    assert grasps
    for grasp in grasps:
        assert grasp.grasp_pose.to_np()[2, 3] > lifting_height


def test_without_a_model_the_annotation_offers_its_own_grasps(milk_scene):
    milk = milk_scene.graspable

    grasps = GraspModelLibrary().grasp_candidates(
        milk, type(milk_scene.arm.end_effector)
    )

    assert [grasp.grasp_pose.to_np().tolist() for grasp in grasps] == [
        grasp.grasp_pose.to_np().tolist() for grasp in milk.grasp_candidates()
    ]


# %% the whole pipeline


@simulates_physics
def test_the_pipeline_stores_attempts_and_a_verified_model(database):
    pipeline = GraspLearningPipeline(
        experiment=PickUpExperiment(
            scene=PickUpScene(object_description=PickUpObject.MILK.value),
            time_limit=timedelta(seconds=20),
        ),
        database=database,
        number_of_attempts=12,
        number_of_verifying_attempts=3,
    )

    model = pipeline.run()

    attempts = database.attempts(pipeline.task)
    sources = [attempt.source for attempt in attempts]
    assert model.number_of_attempts == sources.count(GraspSource.STATED_REGIONS)
    assert 0 < sources.count(GraspSource.LEARNED_MODEL) <= 3
    assert 0.0 <= model.verified_lift_rate <= 1.0
    assert database.model_library().models[-1].task == pipeline.task

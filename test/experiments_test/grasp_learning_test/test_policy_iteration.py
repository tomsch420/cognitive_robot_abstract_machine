"""
Iterating a grasp policy until it stops changing, and measuring whether it still does.
"""

from __future__ import annotations

import inspect
import json
import math
from pathlib import Path

import numpy as np
import pytest

from ...pytest_environment import runs_in_continuous_integration
from ..physical_pick_up_test.recorded_part_locator import RecordedPartLocator
from .test_grasp_learning import synthetic_attempts, task_of

from experiments.grasp_learning.convergence import (
    Distance,
    ImportanceWeights,
    Measurements,
    SamplingDensities,
    SamplingPhase,
)
from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.grasp_selection import turned
from experiments.grasp_learning.grasp_variables import grasp_features
from experiments.grasp_learning.models import GraspModelLearner
from experiments.grasp_learning.policy_iteration import (
    PolicyIteration,
    graspable_annotation_types,
)
from experiments.grasp_learning.stopping_rules import FixedRounds, PolicyUnchanged
from experiments.grasp_learning.workers import IterationObject, WorkerPool
from experiments.grasp_learning.records import GraspSource
from experiments.physical_pick_up.objects import PickUpObject
from experiments.physical_pick_up.robots import PickUpRobot
from experiments.physical_pick_up.scene import PickUpScene
from krrood.parametrization.parameterizer import UnderspecifiedParameters
from semantic_digital_twin.part_finding.annotation_part_finder import (
    AnnotationPartFinder,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Mug

simulates_physics = pytest.mark.skipif(
    not runs_in_continuous_integration(), reason="MuJoCo tests only run in CI"
)

# %% fixtures


@pytest.fixture(scope="module")
def milk_scene() -> PickUpScene:
    return PickUpScene(
        object_description=PickUpObject.MILK.value,
        part_finder=AnnotationPartFinder(locator=RecordedPartLocator()),
    )


@pytest.fixture
def database() -> GraspDatabase:
    return GraspDatabase.connect("sqlite:///:memory:")


@pytest.fixture(scope="module")
def milk_circuit(milk_scene):
    model = GraspModelLearner().learn(
        task_of(milk_scene), synthetic_attempts(milk_scene, 200)
    )
    statement = milk_scene.graspable.grasped_part().surface_grasp_statement()
    return model.model_registry().get_model(UnderspecifiedParameters(statement))


# %% grasps as points


def test_a_turned_grasp_keeps_its_azimuth_within_a_full_turn(milk_scene):
    [grasp] = synthetic_attempts(milk_scene, 1)
    grasp = grasp.grasp

    result = turned(grasp, 2 * math.pi + 0.5)

    assert result.azimuth == pytest.approx(math.fmod(grasp.azimuth + 0.5, 2 * math.pi))
    assert result is not grasp


def test_grasps_from_either_side_of_zero_azimuth_lie_close(milk_scene):
    [attempt] = synthetic_attempts(milk_scene, 1)
    near_zero = turned(attempt.grasp, -attempt.grasp.azimuth + 0.01)
    near_full_turn = turned(attempt.grasp, -attempt.grasp.azimuth - 0.01)

    distance = np.linalg.norm(
        np.subtract(grasp_features(near_zero), grasp_features(near_full_turn))
    )

    assert distance == pytest.approx(2 * math.sin(0.01), abs=1e-9)


# %% telling policies apart


def test_grasps_drawn_from_one_policy_cannot_be_told_apart():
    generator = np.random.default_rng(0)
    rule = PolicyUnchanged(number_of_permutations=200)

    p_value = rule.p_value(
        generator.normal(size=(80, 3)), generator.normal(size=(80, 3)), generator
    )

    assert p_value >= rule.significance


def test_grasps_drawn_from_policies_far_apart_are_told_apart():
    generator = np.random.default_rng(0)
    rule = PolicyUnchanged(number_of_permutations=200)

    p_value = rule.p_value(
        generator.normal(size=(80, 3)), generator.normal(3.0, size=(80, 3)), generator
    )

    assert p_value < rule.significance


# %% distances between models


def test_a_model_is_at_no_distance_from_itself(milk_circuit):
    distance = Distance.joint_l1(milk_circuit, milk_circuit, 2000)

    assert distance.value == pytest.approx(0.0, abs=1e-12)


def test_measurements_converge_only_with_both_distances_within_their_floors():
    small = Distance(value=0.01, standard_error=0.0)
    floor = Distance(value=0.05, standard_error=0.0)
    large = Distance(value=0.5, standard_error=0.0)

    def measurements(conditional, joint) -> Measurements:
        return Measurements(
            attempts=10,
            effective_sample_size=10.0,
            conditional_l1=conditional,
            conditional_floor=floor,
            weighted_joint_l1=joint,
            weighted_joint_floor=floor,
            unweighted_joint_l1=joint,
        )

    assert measurements(small, small).converged
    assert not measurements(small, large).converged
    assert not measurements(large, small).converged
    assert not measurements(None, None).converged


# %% reweighting attempts to uniformly drawn grasps


def test_attempts_drawn_uniformly_keep_their_weight(milk_scene):
    attempts = dict(enumerate(synthetic_attempts(milk_scene, 30), start=1))
    name = milk_scene.graspable.root.name.name
    phases = [SamplingPhase(name, GraspSource.STATED_REGIONS, None, 1, 30)]
    densities = SamplingDensities(
        parts={name: milk_scene.graspable.grasped_part()}, models=[]
    )

    weights = ImportanceWeights.of(attempts, phases, densities)

    np.testing.assert_allclose(weights.weights, np.ones(30))
    assert weights.effective_sample_size == pytest.approx(30.0)


def test_a_phase_holds_the_attempts_on_its_object_stored_within_it():
    phase = SamplingPhase("milk", GraspSource.STATED_REGIONS, None, 3, 5)

    assert phase.holds(3, "milk")
    assert phase.holds(5, "milk")
    assert not phase.holds(6, "milk")
    assert not phase.holds(4, "bowl")


def test_phases_are_read_back_as_written(tmp_path: Path):
    path = tmp_path / "phases.jsonl"
    phases = [
        SamplingPhase("milk", GraspSource.STATED_REGIONS, None, 1, 4),
        SamplingPhase("milk", GraspSource.LEARNED_MODEL, 0, 5, 9),
    ]

    for phase in phases:
        phase.append_to(path)

    assert SamplingPhase.read_all(path) == phases


# %% stored attempts by their numbers


def test_attempts_are_numbered_in_the_order_they_were_stored(milk_scene, database):
    attempts = synthetic_attempts(milk_scene, 4)
    database.add_attempts(attempts)

    numbered = database.numbered_attempts(task_of(milk_scene))

    assert list(numbered) == sorted(numbered)
    assert len(numbered) == len(attempts)
    assert database.last_attempt_number() == max(numbered)


def test_an_empty_database_has_no_last_attempt(database):
    assert database.last_attempt_number() == 0


# %% the workers


def test_every_worker_tries_grasps_on_one_object_when_there_are_more_workers():
    descriptions = [PickUpObject.MILK.value, PickUpObject.BOWL.value]
    objects = [
        IterationObject(description=description, robot=PickUpRobot.PR2)
        for description in descriptions
    ]
    pool = WorkerPool(objects=objects, size=5, database_uri="sqlite://")

    assert pool.workers_of(objects[0]) == [0, 2, 4]
    assert pool.workers_of(objects[1]) == [1, 3]


def test_objects_share_workers_when_there_are_fewer_workers():
    descriptions = [
        PickUpObject.MILK.value,
        PickUpObject.BOWL.value,
        PickUpObject.CUP.value,
    ]
    objects = [
        IterationObject(description=description, robot=PickUpRobot.PR2)
        for description in descriptions
    ]
    pool = WorkerPool(objects=objects, size=2, database_uri="sqlite://")

    assert [pool.workers_of(iteration_object) for iteration_object in objects] == [
        [0],
        [1],
        [0],
    ]


# %% a new object


def test_a_new_object_is_described_by_its_mesh_annotation_and_mass():
    mesh_file = PickUpObject.MILK.value.mesh_file
    annotation_type = PickUpObject.MILK.value.semantic_annotation_type

    new_object = IterationObject.from_mesh(mesh_file, annotation_type, mass=0.4)

    description = new_object.description
    assert new_object.name == mesh_file.stem
    assert description.mesh_file == mesh_file
    assert description.semantic_annotation_type is annotation_type
    assert description.mass == 0.4
    assert description.convex_decomposition is None


def test_a_new_hollow_object_collides_as_parts_that_leave_it_open():
    new_object = IterationObject.from_mesh(
        PickUpObject.BOWL.value.mesh_file,
        PickUpObject.BOWL.value.semantic_annotation_type,
        mass=0.2,
        hollow=True,
    )

    assert new_object.description.convex_decomposition is not None


def test_an_object_whose_annotation_calls_for_no_parts_needs_no_part_finding_server(
    monkeypatch,
):
    monkeypatch.delenv("SEMANTIC_DIGITAL_TWIN_PART_FINDING_SERVER_URL", raising=False)
    milk = IterationObject(description=PickUpObject.MILK.value)

    scene = milk.scene()

    assert type(scene.graspable) is PickUpObject.MILK.value.semantic_annotation_type


def test_the_command_line_offers_every_annotation_that_can_be_grasped():
    annotation_types = graspable_annotation_types()

    assert annotation_types["Mug"] is Mug
    assert all(
        not inspect.isabstract(annotation_type)
        for annotation_type in annotation_types.values()
    )


# %% the whole iteration


@simulates_physics
def test_an_iteration_stores_the_attempts_and_a_model_of_every_round(tmp_path):
    milk = IterationObject.from_mesh(
        PickUpObject.MILK.value.mesh_file,
        PickUpObject.MILK.value.semantic_annotation_type,
        mass=PickUpObject.MILK.value.mass,
        meters_per_mesh_unit=PickUpObject.MILK.value.meters_per_mesh_unit,
    )
    database_uri = f"sqlite:///{tmp_path / 'grasps.sqlite'}"
    iteration = PolicyIteration(
        objects=[milk],
        database_uri=database_uri,
        results=tmp_path / "results",
        workers=2,
        stopping=FixedRounds(),
        initial_attempts=12,
        attempts_per_round=4,
        maximum_rounds=1,
    )

    model = iteration.run()

    rounds = [
        json.loads(line) for line in iteration.rounds_file.read_text().splitlines()
    ]
    assert [report["round"] for report in rounds] == [0, 1]
    assert model.number_of_attempts == rounds[0]["exploring_attempts"]
    stored = GraspDatabase.connect(database_uri).model_library().models
    assert stored[-1].task == model.task
    first_phase = iteration.phases[0]
    assert (first_phase.source, first_phase.model_index) == (
        GraspSource.STATED_REGIONS,
        None,
    )

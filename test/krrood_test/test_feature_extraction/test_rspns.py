import math

import pytest

from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.entity_query_language.factories import underspecified
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.parametrization.model_registries import RelationalCircuitRegistry
from probabilistic_model.probabilistic_circuit.relational.exceptions import (
    CircuitNotFittedError,
)
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit
from ..dataset import ormatic_interface  # type: ignore
from ..dataset.example_classes import (
    KRROODOrientation,
    KRROODPosition,
    SceneObject,
    SceneObjectType,
    SceneRoom,
)


@pytest.fixture
def scenario():
    objects = [
        SceneObject(type=SceneObjectType.TABLE),
        SceneObject(type=SceneObjectType.CHAIR),
        SceneObject(type=SceneObjectType.CHAIR),
        SceneObject(type=SceneObjectType.CHAIR),
    ]
    room = SceneRoom(
        position=KRROODPosition(x=2.0, y=1.0, z=0.0),
        orientation=KRROODOrientation(x=0.0, y=0.0, z=0.0, w=1.0),
        objects=objects[:3],
    )
    room2 = SceneRoom(
        position=KRROODPosition(x=4.0, y=3.0, z=0.0),
        orientation=KRROODOrientation(x=0.0, y=0.0, z=0.0, w=1.0),
        objects=objects,
    )
    return to_dao(room), to_dao(room2)


@pytest.fixture
def scene_room_relational_circuit(scenario):
    room_dao, room2_dao = scenario
    model = RelationalProbabilisticCircuit(SceneRoom)
    model.fit([room_dao, room2_dao])
    return model


@pytest.fixture
def room_query_4():
    query = underspecified(SceneRoom)(
        position=underspecified(KRROODPosition)(x=..., y=..., z=...),
        orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=...),
        objects=[underspecified(SceneObject)(type=...) for _ in range(4)],
    )
    query.resolve()
    return query


def test_ground_before_fit_raises(room_query_4):
    model = RelationalProbabilisticCircuit(SceneRoom)
    with pytest.raises(CircuitNotFittedError):
        model.ground(room_query_4)


def test_fit_class_circuit_is_valid(scene_room_relational_circuit):
    assert scene_room_relational_circuit.class_probabilistic_circuit is not None
    assert scene_room_relational_circuit.class_probabilistic_circuit.is_valid()


def test_fit_class_circuit_has_room_scalar_variables(scene_room_relational_circuit):
    names = {v.name for v in scene_room_relational_circuit.class_probabilistic_circuit.variables}
    assert "SceneRoom.position.x" in names
    assert "SceneRoom.position.y" in names
    assert "SceneRoom.position.z" in names
    assert "SceneRoom.orientation.x" in names
    assert "SceneRoom.orientation.y" in names
    assert "SceneRoom.orientation.z" in names
    assert "SceneRoom.orientation.w" in names


def test_fit_class_circuit_has_aggregation_variable(scene_room_relational_circuit):
    names = {v.name for v in scene_room_relational_circuit.class_probabilistic_circuit.variables}
    assert "SceneRoomAggregations.total_count()" in names


def test_fit_creates_exchangeable_template_for_objects(scene_room_relational_circuit):
    assert "objects" in scene_room_relational_circuit.exchangeable_distribution_templates
    template = scene_room_relational_circuit.exchangeable_distribution_templates["objects"]
    assert template.template_distribution.class_probabilistic_circuit is not None


def test_fit_exchangeable_template_latent_is_total_count(scene_room_relational_circuit):
    template = scene_room_relational_circuit.exchangeable_distribution_templates["objects"]
    latent_names = {v.name for v in template.latent_variables}
    assert "SceneRoomAggregations.total_count()" in latent_names


def test_fit_exchangeable_template_models_object_type(scene_room_relational_circuit):
    template = scene_room_relational_circuit.exchangeable_distribution_templates["objects"]
    pc = template.template_distribution.class_probabilistic_circuit
    names = {v.name for v in pc.variables}
    assert "type" in names


def test_ground_circuit_is_valid(scene_room_relational_circuit, room_query_4):
    model = scene_room_relational_circuit.ground(room_query_4)
    assert model.is_valid()


def test_ground_has_per_object_type_variables(scene_room_relational_circuit, room_query_4):
    model = scene_room_relational_circuit.ground(room_query_4)
    names = {v.name for v in model.variables}
    for i in range(4):
        assert f"SceneRoom.objects[{i}].type" in names


def test_ground_preserves_room_scalar_variables(scene_room_relational_circuit, room_query_4):
    model = scene_room_relational_circuit.ground(room_query_4)
    names = {v.name for v in model.variables}
    assert "SceneRoom.position.x" in names
    assert "SceneRoom.orientation.w" in names


def test_ground_circuit_contains_sum_unit_for_exchangeable_part(
    scene_room_relational_circuit, room_query_4
):
    model = scene_room_relational_circuit.ground(room_query_4)
    nodes = model.nodes()
    assert any(isinstance(node, SumUnit) for node in nodes)


def test_ground_mixture_has_aggregation_integration_samples_components(
    scene_room_relational_circuit, room_query_4
):
    model = scene_room_relational_circuit.ground(room_query_4)
    sum_units = [node for node in model.nodes() if isinstance(node, SumUnit)]
    expected_count = scene_room_relational_circuit.aggregation_integration_samples
    assert any(len(su.subcircuits) == expected_count for su in sum_units)


def test_ground_mixture_weights_are_uniform(scene_room_relational_circuit, room_query_4):
    model = scene_room_relational_circuit.ground(room_query_4)
    sum_units = [node for node in model.nodes() if isinstance(node, SumUnit)]
    expected_count = scene_room_relational_circuit.aggregation_integration_samples
    mixture = next(su for su in sum_units if len(su.subcircuits) == expected_count)
    expected_log_weight = -math.log(expected_count)
    for log_weight, _ in mixture.log_weighted_subcircuits:
        assert abs(log_weight - expected_log_weight) < 1e-9


def test_ground_with_single_sample_is_valid(scene_room_relational_circuit, room_query_4):
    scene_room_relational_circuit.aggregation_integration_samples = 1
    model = scene_room_relational_circuit.ground(room_query_4)
    assert model.is_valid()
    names = {v.name for v in model.variables}
    for i in range(4):
        assert f"SceneRoom.objects[{i}].type" in names


def test_ground_variable_count_scales_with_query_size(scene_room_relational_circuit):
    query_2 = underspecified(SceneRoom)(
        position=underspecified(KRROODPosition)(x=..., y=..., z=...),
        orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=...),
        objects=[underspecified(SceneObject)(type=...) for _ in range(2)],
    )
    query_2.resolve()
    query_4 = underspecified(SceneRoom)(
        position=underspecified(KRROODPosition)(x=..., y=..., z=...),
        orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=...),
        objects=[underspecified(SceneObject)(type=...) for _ in range(4)],
    )
    query_4.resolve()
    assert (
        len(scene_room_relational_circuit.ground(query_4).variables)
        > len(scene_room_relational_circuit.ground(query_2).variables)
    )


def test_probabilistic_backend_samples_scene_room(scene_room_relational_circuit, scenario):
    """RelationalCircuitRegistry integrates with ProbabilisticBackend to produce typed samples."""
    registry = RelationalCircuitRegistry(scene_room_relational_circuit)
    backend = ProbabilisticBackend(registry, number_of_samples=5)
    query = underspecified(SceneRoom)(
        position=underspecified(KRROODPosition)(x=..., y=..., z=...),
        orientation=underspecified(KRROODOrientation)(x=..., y=..., z=..., w=...),
        objects=[underspecified(SceneObject)(type=...) for _ in range(3)],
    )
    query.resolve()
    samples = list(backend.evaluate(query))
    assert len(samples) == 5
    for sample in samples:
        assert isinstance(sample, SceneRoom)
        assert len(sample.objects) == 3

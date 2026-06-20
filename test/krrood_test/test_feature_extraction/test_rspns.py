import pytest

from krrood.entity_query_language.factories import underspecified
from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.exceptions import (
    CircuitNotFittedError,
)
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit
from ..dataset import ormatic_interface  # type: ignore
from ..dataset.example_classes import (
    BookSceneItem,
    KRROODOrientation,
    KRROODPosition,
    LampSceneItem,
    LibraryRoom,
    PolymorphicSceneItem,
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
def rpc(scenario):
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


def test_fit_class_circuit_is_valid(rpc):
    assert rpc.class_probabilistic_circuit is not None
    assert rpc.class_probabilistic_circuit.is_valid()


def test_fit_class_circuit_has_room_scalar_variables(rpc):
    names = {v.name for v in rpc.class_probabilistic_circuit.variables}
    assert "SceneRoom.position.x" in names
    assert "SceneRoom.position.y" in names
    assert "SceneRoom.position.z" in names
    assert "SceneRoom.orientation.x" in names
    assert "SceneRoom.orientation.y" in names
    assert "SceneRoom.orientation.z" in names
    assert "SceneRoom.orientation.w" in names


def test_fit_class_circuit_has_aggregation_variable(rpc):
    names = {v.name for v in rpc.class_probabilistic_circuit.variables}
    assert "SceneRoomAggregations.total_count()" in names


def test_fit_creates_exchangeable_template_for_objects(rpc):
    assert "objects" in rpc.exchangeable_distribution_templates
    template = rpc.exchangeable_distribution_templates["objects"]
    assert template.template_distribution.class_probabilistic_circuit is not None


def test_fit_exchangeable_template_latent_is_total_count(rpc):
    template = rpc.exchangeable_distribution_templates["objects"]
    latent_names = {v.name for v in template.latent_variables}
    assert "SceneRoomAggregations.total_count()" in latent_names


def test_fit_exchangeable_template_models_object_type(rpc):
    template = rpc.exchangeable_distribution_templates["objects"]
    pc = template.template_distribution.class_probabilistic_circuit
    names = {v.name for v in pc.variables}
    assert "type" in names


def test_ground_circuit_is_valid(rpc, room_query_4):
    model = rpc.ground(room_query_4)
    assert model.is_valid()


def test_ground_has_per_object_type_variables(rpc, room_query_4):
    model = rpc.ground(room_query_4)
    names = {v.name for v in model.variables}
    for i in range(4):
        assert f"SceneRoom.objects[{i}].type" in names


def test_ground_preserves_room_scalar_variables(rpc, room_query_4):
    model = rpc.ground(room_query_4)
    names = {v.name for v in model.variables}
    assert "SceneRoom.position.x" in names
    assert "SceneRoom.orientation.w" in names


def test_ground_circuit_contains_sum_unit_for_exchangeable_part(rpc, room_query_4):
    model = rpc.ground(room_query_4)
    nodes = model.nodes()
    assert any(isinstance(node, SumUnit) for node in nodes)


def test_ground_mixture_has_n_samples_components(rpc, room_query_4):
    model = rpc.ground(room_query_4)
    sum_units = [node for node in model.nodes() if isinstance(node, SumUnit)]
    # The top-level SumUnit wrapping the grounded exchangeable parts has n_samples children.
    assert any(len(su.subcircuits) == rpc.n_samples for su in sum_units)


def test_ground_mixture_weights_are_uniform(rpc, room_query_4):
    import math
    model = rpc.ground(room_query_4)
    sum_units = [node for node in model.nodes() if isinstance(node, SumUnit)]
    mixture = next(su for su in sum_units if len(su.subcircuits) == rpc.n_samples)
    expected = -math.log(rpc.n_samples)
    for log_weight, _ in mixture.log_weighted_subcircuits:
        assert abs(log_weight - expected) < 1e-9


def test_ground_with_single_sample_is_valid(rpc, room_query_4):
    rpc.n_samples = 1
    model = rpc.ground(room_query_4)
    assert model.is_valid()
    names = {v.name for v in model.variables}
    for i in range(4):
        assert f"SceneRoom.objects[{i}].type" in names


def test_ground_variable_count_scales_with_query_size(rpc):
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
    assert len(rpc.ground(query_4).variables) > len(rpc.ground(query_2).variables)


# ── Polymorphic RSPN tests ──────────────────────────────────────────────────


@pytest.fixture
def polymorphic_items():
    """Two BookSceneItem and two LampSceneItem DAO instances."""
    books = [
        BookSceneItem(size=0.5, pages=200),
        BookSceneItem(size=0.8, pages=350),
    ]
    lamps = [
        LampSceneItem(size=1.0, brightness=100.0),
        LampSceneItem(size=1.2, brightness=150.0),
    ]
    return [to_dao(item) for item in books + lamps]


@pytest.fixture
def polymorphic_rpc(polymorphic_items):
    """RelationalProbabilisticCircuit fitted on a polymorphic collection."""
    model = RelationalProbabilisticCircuit(PolymorphicSceneItem)
    model.fit(polymorphic_items)
    return model


@pytest.fixture
def library_rpc():
    """RelationalProbabilisticCircuit fitted on LibraryRoom with polymorphic items."""
    rooms = [
        LibraryRoom(
            position=KRROODPosition(x=1.0, y=2.0, z=0.0),
            items=[
                BookSceneItem(size=0.5, pages=200),
                LampSceneItem(size=1.0, brightness=100.0),
            ],
        ),
        LibraryRoom(
            position=KRROODPosition(x=3.0, y=4.0, z=0.0),
            items=[
                BookSceneItem(size=0.8, pages=350),
                BookSceneItem(size=0.6, pages=150),
                LampSceneItem(size=1.2, brightness=150.0),
            ],
        ),
    ]
    model = RelationalProbabilisticCircuit(LibraryRoom)
    model.fit([to_dao(room) for room in rooms])
    return model


@pytest.fixture
def library_query():
    """Query for a LibraryRoom with one book and one lamp."""
    query = underspecified(LibraryRoom)(
        position=underspecified(KRROODPosition)(x=..., y=..., z=...),
        items=[
            underspecified(PolymorphicSceneItem)(size=...),
            underspecified(PolymorphicSceneItem)(size=...),
        ],
    )
    query.resolve()
    return query


def test_polymorphic_fit_creates_sub_type_circuits(polymorphic_rpc):
    assert len(polymorphic_rpc.sub_type_circuits) == 2
    assert BookSceneItem in polymorphic_rpc.sub_type_circuits
    assert LampSceneItem in polymorphic_rpc.sub_type_circuits


def test_polymorphic_fit_log_weights_sum_to_zero(polymorphic_rpc):
    import math
    total = sum(math.exp(w) for w in polymorphic_rpc.log_type_weights.values())
    assert abs(total - 1.0) < 1e-9


def test_polymorphic_fit_weights_reflect_frequencies(polymorphic_rpc):
    import math
    # 2 books out of 4 → weight 0.5 for each type
    for log_w in polymorphic_rpc.log_type_weights.values():
        assert abs(math.exp(log_w) - 0.5) < 1e-9


def test_polymorphic_ground_returns_sum_unit(polymorphic_rpc):
    query = underspecified(PolymorphicSceneItem)(size=...)
    query.resolve()
    model = polymorphic_rpc.ground(query)
    assert any(isinstance(node, SumUnit) for node in model.nodes())


def test_polymorphic_ground_has_all_concrete_type_variables(polymorphic_rpc):
    query = underspecified(PolymorphicSceneItem)(size=...)
    query.resolve()
    model = polymorphic_rpc.ground(query)
    names = {v.name for v in model.variables}
    assert "size" in names
    assert "pages" in names
    assert "brightness" in names


def test_polymorphic_ground_circuit_is_valid(polymorphic_rpc):
    query = underspecified(PolymorphicSceneItem)(size=...)
    query.resolve()
    assert polymorphic_rpc.ground(query).is_valid()


def test_library_room_rpc_has_polymorphic_template(library_rpc):
    template = library_rpc.exchangeable_distribution_templates["items"]
    assert len(template.template_distribution.sub_type_circuits) == 2


def test_library_room_ground_is_valid(library_rpc, library_query):
    model = library_rpc.ground(library_query)
    assert model.is_valid()


def test_library_room_ground_has_item_variables(library_rpc, library_query):
    model = library_rpc.ground(library_query)
    names = {v.name for v in model.variables}
    # Each item in the query gets its own indexed variables
    assert any("items" in name for name in names)

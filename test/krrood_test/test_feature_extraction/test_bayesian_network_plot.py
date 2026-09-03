import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pytest

from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.bayesian_network_plot import (
    build_bayesian_network,
    plot_circuit_as_bayesian_network,
)
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from ..dataset import ormatic_interface  # noqa: F401  (registers the DAO mappings)
from ..dataset.example_classes import (
    KRROODOrientation,
    KRROODPosition,
    SceneObject,
    SceneObjectType,
    SceneRoom,
)


@pytest.fixture
def rpc() -> RelationalProbabilisticCircuit:
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
    model = RelationalProbabilisticCircuit(SceneRoom)
    model.fit([to_dao(room), to_dao(room2)])
    return model


def test_bayesian_network_has_one_node_per_real_variable(rpc):
    circuit = rpc.class_probabilistic_circuit
    structure = build_bayesian_network(circuit)
    variable_ids = {node.id for node in structure.nodes.values() if node.kind == "variable"}
    expected_ids = {f"variable::{variable.name}" for variable in circuit.variables}
    assert variable_ids == expected_ids


def test_bayesian_network_latent_cardinality_matches_sum_unit_children(rpc):
    circuit = rpc.class_probabilistic_circuit
    structure = build_bayesian_network(circuit)
    sum_unit_child_counts = sorted(
        len(node.subcircuits) for node in circuit.nodes() if node.__class__.__name__ == "SumUnit"
    )
    latent_cardinalities = sorted(
        node.cardinality for node in structure.nodes.values() if node.kind == "latent"
    )
    assert latent_cardinalities == sum_unit_child_counts


def test_bayesian_network_every_variable_has_a_parent(rpc):
    circuit = rpc.class_probabilistic_circuit
    structure = build_bayesian_network(circuit)
    children_with_parent = {child_id for _, child_id in structure.edges}
    variable_ids = {node.id for node in structure.nodes.values() if node.kind == "variable"}
    assert variable_ids.issubset(children_with_parent)


def test_plot_circuit_as_bayesian_network_renders_scene_room(rpc, tmp_path):
    circuit = rpc.class_probabilistic_circuit
    figure = plot_circuit_as_bayesian_network(circuit, class_label="SceneRoom")
    output_path = tmp_path / "scene_room_bn.png"
    figure.savefig(output_path, dpi=200)
    plt.close(figure)
    assert output_path.exists()
    assert output_path.stat().st_size > 0

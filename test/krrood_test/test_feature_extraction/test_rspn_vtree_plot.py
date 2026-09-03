import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pytest

from krrood.ormatic.data_access_objects.helper import to_dao
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.relational.vtree_plot import (
    annotate_complexity,
    build_vtree,
    plot_relational_vtree,
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


def test_build_vtree_is_bounded_by_variable_count(rpc):
    vtree = build_vtree(rpc.class_probabilistic_circuit.variables)
    leaves = [node for node in _flatten(vtree) if node.is_leaf]
    assert len(leaves) == len(rpc.class_probabilistic_circuit.variables)


def test_root_complexity_covers_whole_circuit(rpc):
    vtree = build_vtree(rpc.class_probabilistic_circuit.variables)
    annotate_complexity(vtree, rpc.class_probabilistic_circuit)
    scoped_node_count = sum(
        1 for node in rpc.class_probabilistic_circuit.nodes() if len(node.variables) > 0
    )
    assert vtree.complexity == scoped_node_count


def test_plot_relational_vtree_renders_scene_room(rpc, tmp_path):
    figure = plot_relational_vtree(rpc)
    output_path = tmp_path / "relational_vtree.png"
    figure.savefig(output_path, dpi=200)
    plt.close(figure)
    assert output_path.exists()
    assert output_path.stat().st_size > 0


def _flatten(vtree):
    yield vtree
    for child in vtree.children:
        yield from _flatten(child)

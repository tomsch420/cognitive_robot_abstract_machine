from __future__ import annotations

import inspect
from dataclasses import dataclass
from typing import List, Tuple, Dict, Type, Optional, Any, TYPE_CHECKING
import numpy as np
import tqdm

from krrood.adapters.json_serializer import recursive_subclasses
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    Unit,
    ProbabilisticCircuit as RustworkxProbabilisticCircuit,
)
from random_events.variable import Variable
from sortedcontainers import SortedSet

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.numpy.layer import Layer
    from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import ProbabilisticCircuit


@dataclass
class RustworkxLayerConverter:
    """
    Class used for conversion from a probabilistic circuit in rustworkx to a layered
    circuit in numpy.
    """

    layer: Layer
    nodes: List[Unit]
    hash_remap: Dict[int, int]


def inverse_class_of(clazz: Type[Unit]) -> Type[Layer]:
    """
    Get the layered circuit layer class for a rustworkx unit class.
    """
    from probabilistic_model.probabilistic_circuit.numpy.layer import Layer
    from probabilistic_model.probabilistic_circuit.numpy.input_layer import (
        DiscreteLayer,
        GaussianLayer,
        UniformLayer,
        DiracDeltaLayer,
        TruncatedGaussianLayer,
    )

    for subclass in recursive_subclasses(Layer):
        if not inspect.isabstract(subclass):
            if issubclass(clazz, subclass.rustworkx_classes()):
                return subclass

    raise TypeError(f"Could not find class for {clazz}")


def create_layers_from_nodes(
    nodes: List[Unit],
    child_layers: List[RustworkxLayerConverter],
    progress_bar: bool = True,
) -> List[RustworkxLayerConverter]:
    """
    Create a layer from a list of nodes.
    """
    result = []

    unique_types = set(
        type(node) if not node.is_leaf else type(node.distribution)
        for node in nodes
    )
    for unique_type in unique_types:
        nodes_of_current_type = [
            node
            for node in nodes
            if (
                isinstance(node, unique_type)
                if not node.is_leaf
                else isinstance(node.distribution, unique_type)
            )
        ]

        if nodes[0].is_leaf:
            unique_type = type(nodes_of_current_type[0].distribution)

        layer_type = inverse_class_of(unique_type)

        scopes = [tuple(node.variables) for node in nodes_of_current_type]
        unique_scopes = set(scopes)
        for scope in unique_scopes:
            nodes_of_current_type_and_scope = [
                node
                for node in nodes_of_current_type
                if tuple(node.variables) == scope
            ]

            layer = layer_type.create_layer_from_nodes_with_same_type_and_scope(
                nodes_of_current_type_and_scope, child_layers, progress_bar
            )
            result.append(layer)

    return result


def from_rustworkx(
    pc: RustworkxProbabilisticCircuit, progress_bar: bool = False
) -> ProbabilisticCircuit:
    """
    Convert a probabilistic circuit to a layered circuit.

    The result expresses the same distribution as `pc`.

    :param pc: The probabilistic circuit.
    :param progress_bar: Whether to show a progress bar.
    :return: The layered circuit.
    """
    from probabilistic_model.probabilistic_circuit.numpy.probabilistic_circuit import (
        ProbabilisticCircuit,
    )

    # group nodes by depth
    layer_to_nodes_map = {index: layer for index, layer in enumerate(pc.layers)}
    reversed_layers_to_nodes_map = dict(reversed(layer_to_nodes_map.items()))

    # create layers from nodes
    child_layers: List[RustworkxLayerConverter] = []
    for layer_index, nodes in (
        tqdm.tqdm(reversed_layers_to_nodes_map.items(), desc="Creating Layers")
        if progress_bar
        else reversed_layers_to_nodes_map.items()
    ):
        child_layers = create_layers_from_nodes(nodes, child_layers, progress_bar)
    root = child_layers[0].layer

    return ProbabilisticCircuit(pc.variables, root)


def to_rustworkx(
    pc: ProbabilisticCircuit, progress_bar: bool = True
) -> RustworkxProbabilisticCircuit:
    """
    Convert the probabilistic circuit to a rustworkx graph.

    :param pc: The probabilistic circuit.
    :param progress_bar: Whether to show a progress bar.
    :return: The rustworkx graph.
    """
    if progress_bar:
        number_of_edges = pc.root.number_of_components
        progress_bar = tqdm.tqdm(total=number_of_edges, desc="Converting to rx")
    else:
        progress_bar = None
    result = RustworkxProbabilisticCircuit()
    pc.root.to_rustworkx(pc.variables, result, progress_bar)
    return result

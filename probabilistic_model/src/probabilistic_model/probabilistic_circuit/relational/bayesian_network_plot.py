"""
Bayesian-network structure view of a single :class:`ProbabilisticCircuit`.

Reduces the circuit to the classical sum-product-network-to-Bayesian-network
correspondence: every sum unit becomes a latent categorical variable, every product
unit disappears and instead passes its own parent straight through to each of its
children, and every leaf becomes a node for the real variable it models, wired to the
nearest enclosing latent. No parameters (weights, distribution parameters) are exported
— only the dependency structure this recursion induces, built directly on the
framework's own :class:`~probabilistic_model.bayesian_network.bayesian_network.BayesianNetwork`
graph rather than a bespoke structure: a latent's node reuses
:attr:`SumUnit.latent_variable` as-is, so its cardinality is the size of that
variable's own domain, not a hand-computed count.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
from random_events.variable import Symbolic, Variable
from typing_extensions import Dict, List, Optional, Tuple

from probabilistic_model.bayesian_network.bayesian_network import BayesianNetwork, Node
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
    ProductUnit,
    SumUnit,
    Unit,
)

NODE_SPACING = 1.9
LEVEL_SPACING = 1.35
LATENT_RADIUS = 0.36
VARIABLE_BOX_WIDTH = 1.6
VARIABLE_BOX_HEIGHT = 0.5
INCHES_PER_UNIT_X = 0.85
INCHES_PER_UNIT_Y = 0.95
"""Figure size scales with layout extent at this ratio, matching the vtree renderer's
approach: a fixed point size never has to fit into a box that shrinks as the diagram
grows."""

LATENT_COLOR = "#bf6a2e"
"""Same accent used for latent/bridge variables in the vtree view, reused here since
every latent in this diagram is, structurally, the same kind of thing: a sum unit's
mixture indicator."""
VARIABLE_COLOR = "#3b6ea5"
"""Same root-panel blue used in the vtree view, reused here for the real variables of
the circuit."""


@dataclass(eq=False)
class LatentNode(Node):
    """
    A sum unit's latent variable, carried structure-only: no conditional probability
    distribution is attached, since this view exports dependency structure, not
    parameters. ``variable`` is the very same :class:`Symbolic` produced by
    :attr:`SumUnit.latent_variable`, so its domain size already is the correct
    cardinality — the number of subcircuits that sum unit mixes over.
    """

    variable: Optional[Symbolic] = None
    __hash__ = Node.__hash__

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return (self.variable,)

    @property
    def cardinality(self) -> int:
        return len(self.variable.domain.hash_map)

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        raise NotImplementedError(
            "This Bayesian network carries structure only; it was never meant to be "
            "converted back into a parameterized circuit."
        )


@dataclass(eq=False)
class RealVariableNode(Node):
    """A leaf's real variable, carried structure-only."""

    variable: Optional[Variable] = None
    __hash__ = Node.__hash__

    @property
    def variables(self) -> Tuple[Variable, ...]:
        return (self.variable,)

    def as_probabilistic_circuit(self, result: ProbabilisticCircuit):
        raise NotImplementedError(
            "This Bayesian network carries structure only; it was never meant to be "
            "converted back into a parameterized circuit."
        )


def build_bayesian_network(circuit: ProbabilisticCircuit) -> BayesianNetwork:
    """
    Reduce ``circuit`` to its Bayesian-network structure.

    Walks the circuit from its root. A sum unit introduces a fresh :class:`LatentNode`
    (its variable is :attr:`SumUnit.latent_variable`, unchanged) and becomes the parent
    passed to each of its subcircuits. A product unit introduces no node and simply
    forwards its own parent to each of its subcircuits, so a product's children all
    depend directly on the same enclosing latent (or on nothing, if the product itself
    has no enclosing sum). A leaf contributes one :class:`RealVariableNode` per real
    variable it models, with an edge from the parent latent, if any.

    :param circuit: The circuit to reduce. Its ``root`` must exist.
    :return: The induced Bayesian network, built on the framework's own graph classes.
    """
    bn = BayesianNetwork()
    variable_nodes: Dict[str, RealVariableNode] = {}

    def walk(unit: Unit, parent_node: Optional[Node]) -> None:
        if unit.is_leaf:
            for variable in unit.variables:
                node = variable_nodes.get(variable.name)
                if node is None:
                    node = RealVariableNode(variable=variable, bayesian_network=bn)
                    variable_nodes[variable.name] = node
                if parent_node is not None and not bn.has_edge(parent_node, node):
                    bn.add_edge(parent_node, node)
            return
        if isinstance(unit, ProductUnit):
            for subcircuit in unit.subcircuits:
                walk(subcircuit, parent_node)
            return
        if isinstance(unit, SumUnit):
            latent_node = LatentNode(variable=unit.latent_variable, bayesian_network=bn)
            if parent_node is not None:
                bn.add_edge(parent_node, latent_node)
            for subcircuit in unit.subcircuits:
                walk(subcircuit, latent_node)
            return
        raise TypeError(f"Unexpected unit type in circuit: {type(unit)!r}")

    walk(circuit.root, None)
    return bn


def _assign_layers(bn: BayesianNetwork) -> Dict[int, int]:
    """Longest-path-from-root layering via Kahn's algorithm, keyed by node index, so a
    node reachable through paths of different lengths (a variable modeled under more
    than one mixture branch) is always drawn below every one of its parents."""
    indegree = {node.index: bn.in_degree(node) for node in bn.nodes()}
    children: Dict[int, List[int]] = defaultdict(list)
    for parent, child in bn.edges():
        children[parent.index].append(child.index)

    layer: Dict[int, int] = {}
    queue = deque(index for index, degree in indegree.items() if degree == 0)
    for index in queue:
        layer[index] = 0

    remaining_indegree = dict(indegree)
    while queue:
        index = queue.popleft()
        for child_index in children[index]:
            layer[child_index] = max(layer.get(child_index, 0), layer[index] + 1)
            remaining_indegree[child_index] -= 1
            if remaining_indegree[child_index] == 0:
                queue.append(child_index)
    return layer


def _wrap_label(label: str, max_chars: int = 14) -> str:
    if len(label) <= max_chars:
        return label
    midpoint = len(label) // 2
    breakpoints = [index for index, char in enumerate(label) if char in "._"]
    if not breakpoints:
        return label
    split_at = min(breakpoints, key=lambda index: abs(index - midpoint))
    return f"{label[: split_at + 1]}\n{label[split_at + 1 :]}"


def plot_circuit_as_bayesian_network(
    circuit: ProbabilisticCircuit, class_label: Optional[str] = None
) -> Figure:
    """
    Render ``circuit`` as its induced Bayesian network: one node per real variable, one
    latent node per sum unit labeled with its real cardinality, structure only (no
    weights, no distribution parameters).

    :param circuit: The circuit to render.
    :param class_label: Optional title naming the class this circuit belongs to.
    :return: The rendered matplotlib figure.
    """
    bn = build_bayesian_network(circuit)
    layer = _assign_layers(bn)

    nodes_by_layer: Dict[int, List[Node]] = defaultdict(list)
    for node in bn.nodes():
        nodes_by_layer[layer[node.index]].append(node)

    positions: Dict[int, Tuple[float, float]] = {}
    for depth, nodes in nodes_by_layer.items():
        count = len(nodes)
        y = -depth * LEVEL_SPACING
        for index, node in enumerate(nodes):
            x = (index - (count - 1) / 2) * NODE_SPACING
            positions[node.index] = (x, y)

    latent_display_names: Dict[int, str] = {
        node.index: f"Z{sequence}"
        for sequence, node in enumerate(
            (node for node in bn.nodes() if isinstance(node, LatentNode)), start=1
        )
    }

    fig, ax = plt.subplots()

    for parent, child in bn.edges():
        x0, y0 = positions[parent.index]
        x1, y1 = positions[child.index]
        target_gap = LATENT_RADIUS if isinstance(child, LatentNode) else VARIABLE_BOX_HEIGHT / 2
        ax.add_patch(
            FancyArrowPatch(
                (x0, y0 - LATENT_RADIUS),
                (x1, y1 + target_gap),
                arrowstyle="-|>",
                mutation_scale=9,
                color=LATENT_COLOR,
                alpha=0.55,
                linewidth=1.0,
                zorder=1,
            )
        )

    for node in bn.nodes():
        x, y = positions[node.index]
        if isinstance(node, LatentNode):
            ax.add_patch(
                Circle(
                    (x, y),
                    LATENT_RADIUS,
                    facecolor=mcolors.to_rgba(LATENT_COLOR, alpha=0.15),
                    edgecolor=LATENT_COLOR,
                    linewidth=1.3,
                    zorder=2,
                )
            )
            label = f"{latent_display_names[node.index]}\ncard={node.cardinality}"
            ax.text(
                x, y, label, ha="center", va="center", fontsize=8,
                color=LATENT_COLOR, fontweight="bold", zorder=3,
            )
        else:
            label = _wrap_label(node.variable.name)
            ax.add_patch(
                FancyBboxPatch(
                    (x - VARIABLE_BOX_WIDTH / 2, y - VARIABLE_BOX_HEIGHT / 2),
                    VARIABLE_BOX_WIDTH,
                    VARIABLE_BOX_HEIGHT,
                    boxstyle="round,pad=0,rounding_size=0.07",
                    facecolor=mcolors.to_rgba(VARIABLE_COLOR, alpha=0.08),
                    edgecolor=VARIABLE_COLOR,
                    linewidth=1.2,
                    zorder=2,
                )
            )
            ax.text(
                x, y, label, ha="center", va="center", fontsize=7.5,
                color=VARIABLE_COLOR, zorder=3,
            )

    latent_total = len(latent_display_names)
    variable_total = len(bn.nodes()) - latent_total
    title = f"{latent_total} latent · {variable_total} variables"
    if class_label:
        title = f"{class_label} — {title}"
    ax.text(
        0.0, LEVEL_SPACING * 0.4, title, ha="center", va="bottom",
        fontsize=10, fontweight="bold", color="0.25",
        transform=ax.transData,
    )

    ax.axis("off")
    ax.set_aspect("equal")

    xs = [x for x, _ in positions.values()]
    ys = [y for _, y in positions.values()]
    x_margin = VARIABLE_BOX_WIDTH / 2 + 0.3
    y_margin = LEVEL_SPACING * 0.6
    ax.set_xlim(min(xs) - x_margin, max(xs) + x_margin)
    ax.set_ylim(min(ys) - y_margin, max(ys) + y_margin)

    width = min(max((max(xs) - min(xs) + 2 * x_margin) * INCHES_PER_UNIT_X, 6.0), 22.0)
    height = min(max((max(ys) - min(ys) + 2 * y_margin) * INCHES_PER_UNIT_Y, 3.5), 14.0)
    fig.set_size_inches(width, height)
    fig.tight_layout()
    return fig

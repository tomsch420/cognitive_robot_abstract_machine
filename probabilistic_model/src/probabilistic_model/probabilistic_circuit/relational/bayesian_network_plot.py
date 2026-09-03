"""
Bayesian-network structure view of a single :class:`ProbabilisticCircuit`.

Reduces the circuit to the classical sum-product-network-to-Bayesian-network
correspondence: every sum unit becomes a latent categorical variable (its cardinality
is the number of subcircuits it mixes over), every product unit disappears and instead
passes its own parent straight through to each of its children, and every leaf becomes
a node for the real variable it models, wired to the nearest enclosing latent. No
parameters (weights, distribution parameters) are exported — only the dependency
structure this recursion induces.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch
from typing_extensions import Dict, List, Optional, Tuple

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


@dataclass
class BNNode:
    """One node of the reduced Bayesian network: either a real variable or a sum
    unit's latent."""

    id: str
    kind: str
    """``"variable"`` or ``"latent"``."""
    label: str
    cardinality: Optional[int] = None
    """Set only for latent nodes: how many subcircuits the originating sum unit mixes
    over."""


@dataclass
class BNStructure:
    """The structure-only Bayesian network induced by a circuit: nodes plus a
    deduplicated, ordered edge list (parent id, child id)."""

    nodes: Dict[str, BNNode] = field(default_factory=dict)
    edges: List[Tuple[str, str]] = field(default_factory=list)


def build_bayesian_network(circuit: ProbabilisticCircuit) -> BNStructure:
    """
    Reduce ``circuit`` to its Bayesian-network structure.

    Walks the circuit from its root. A sum unit introduces a fresh latent node and
    becomes the parent passed to each of its subcircuits. A product unit introduces no
    node and simply forwards its own parent to each of its subcircuits, so a product's
    children all depend directly on the same enclosing latent (or on nothing, if the
    product itself has no enclosing sum). A leaf contributes one node per real variable
    it models, with an edge from the parent latent, if any.

    :param circuit: The circuit to reduce. Its ``root`` must exist.
    :return: The induced Bayesian-network structure.
    """
    structure = BNStructure()
    seen_edges = set()
    latent_count = 0

    def add_edge(parent_id: Optional[str], child_id: str) -> None:
        if parent_id is None or (parent_id, child_id) in seen_edges:
            return
        seen_edges.add((parent_id, child_id))
        structure.edges.append((parent_id, child_id))

    def walk(unit: Unit, parent_id: Optional[str]) -> None:
        nonlocal latent_count
        if unit.is_leaf:
            for variable in unit.variables:
                variable_id = f"variable::{variable.name}"
                structure.nodes.setdefault(
                    variable_id, BNNode(id=variable_id, kind="variable", label=variable.name)
                )
                add_edge(parent_id, variable_id)
            return
        if isinstance(unit, ProductUnit):
            for subcircuit in unit.subcircuits:
                walk(subcircuit, parent_id)
            return
        if isinstance(unit, SumUnit):
            latent_count += 1
            latent_id = f"latent::{latent_count}"
            structure.nodes[latent_id] = BNNode(
                id=latent_id,
                kind="latent",
                label=f"Z{latent_count}",
                cardinality=len(unit.subcircuits),
            )
            add_edge(parent_id, latent_id)
            for subcircuit in unit.subcircuits:
                walk(subcircuit, latent_id)
            return
        raise TypeError(f"Unexpected unit type in circuit: {type(unit)!r}")

    walk(circuit.root, None)
    return structure


def _assign_layers(structure: BNStructure) -> Dict[str, int]:
    """Longest-path-from-root layering via Kahn's algorithm, so a node that is
    reachable through paths of different lengths (a variable modeled under more than
    one mixture branch) is always drawn below every one of its parents."""
    indegree = {node_id: 0 for node_id in structure.nodes}
    children: Dict[str, List[str]] = defaultdict(list)
    for parent_id, child_id in structure.edges:
        children[parent_id].append(child_id)
        indegree[child_id] += 1

    layer: Dict[str, int] = {}
    queue = deque(node_id for node_id, degree in indegree.items() if degree == 0)
    for node_id in queue:
        layer[node_id] = 0

    remaining_indegree = dict(indegree)
    while queue:
        node_id = queue.popleft()
        for child_id in children[node_id]:
            layer[child_id] = max(layer.get(child_id, 0), layer[node_id] + 1)
            remaining_indegree[child_id] -= 1
            if remaining_indegree[child_id] == 0:
                queue.append(child_id)
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
    Render ``circuit`` as its induced Bayesian network: one node per real variable,
    one latent node per sum unit, structure only (no weights, no distribution
    parameters).

    :param circuit: The circuit to render.
    :param class_label: Optional title naming the class this circuit belongs to.
    :return: The rendered matplotlib figure.
    """
    structure = build_bayesian_network(circuit)
    layer = _assign_layers(structure)

    nodes_by_layer: Dict[int, List[str]] = defaultdict(list)
    for node_id in structure.nodes:
        nodes_by_layer[layer[node_id]].append(node_id)

    positions: Dict[str, Tuple[float, float]] = {}
    for depth, node_ids in nodes_by_layer.items():
        count = len(node_ids)
        y = -depth * LEVEL_SPACING
        for index, node_id in enumerate(node_ids):
            x = (index - (count - 1) / 2) * NODE_SPACING
            positions[node_id] = (x, y)

    fig, ax = plt.subplots()

    for parent_id, child_id in structure.edges:
        x0, y0 = positions[parent_id]
        x1, y1 = positions[child_id]
        child_kind = structure.nodes[child_id].kind
        target_gap = LATENT_RADIUS if child_kind == "latent" else VARIABLE_BOX_HEIGHT / 2
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

    for node_id, node in structure.nodes.items():
        x, y = positions[node_id]
        if node.kind == "latent":
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
            label = f"{node.label}\n×{node.cardinality}"
            ax.text(
                x, y, label, ha="center", va="center", fontsize=8,
                color=LATENT_COLOR, fontweight="bold", zorder=3,
            )
        else:
            label = _wrap_label(node.label)
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

    latent_total = sum(1 for node in structure.nodes.values() if node.kind == "latent")
    variable_total = sum(1 for node in structure.nodes.values() if node.kind == "variable")
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

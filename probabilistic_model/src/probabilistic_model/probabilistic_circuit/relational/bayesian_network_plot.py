"""
Bayesian-network structure view of :class:`RelationalProbabilisticCircuit`.

Reduces a single class's own circuit to the classical sum-product-network-to-Bayesian-
network correspondence: every sum unit becomes a latent categorical variable, every
product unit disappears and instead passes its own parent straight through to each of
its children, and every leaf becomes a node for the real variable it models, wired to
the nearest enclosing latent. No parameters (weights, distribution parameters) are
exported — only the dependency structure this recursion induces, built directly on the
framework's own :class:`~probabilistic_model.bayesian_network.bayesian_network.BayesianNetwork`
graph rather than a bespoke structure: a latent's node reuses
:attr:`SumUnit.latent_variable` as-is, so its cardinality is the size of that
variable's own domain, not a hand-computed count.

:func:`plot_relational_bayesian_network` extends this across the relational structure,
Proposal-E style: one bordered panel per class, connected the same way the vtree view's
panels are. Every class a field references gets its own sibling panel rather than a box
nested inside another — an exchangeable (1-to-many) child bridges in with a dashed
"×N" arrow, and a 1-to-1 nested value class (e.g. a ``position: KRROODPosition`` field)
bridges in with a solid arrow instead, since it is exactly one instance and never has
Bayesian-network structure of its own to reduce. A real variable that also serves as an
exchangeable child template's latent (an aggregation statistic, e.g. ``chair_count()``)
is drawn as a diamond rather than a plain box — the same convention the vtree view uses
for the same concept — so it reads as a bridge, not an ordinary per-instance feature.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon
from matplotlib.path import Path
from random_events.variable import Symbolic, Variable
from typing_extensions import Dict, List, Optional, Tuple

from probabilistic_model.bayesian_network.bayesian_network import BayesianNetwork, Node
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.relational.vtree_plot import (
    CLASS_PALETTE,
    _short_label,
)
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

PANEL_HEADER_HEIGHT = 0.85
PANEL_MARGIN = 0.4
PANEL_GAP = 1.3
MIN_PANEL_WIDTH = 1.6
"""Panel geometry, matching the vtree view's constants so the two diagrams read as one
visual family."""

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
                alpha=0.7,
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


@dataclass
class _BNPanelResult:
    width: float
    height: float


@dataclass
class _LayoutSlot:
    """One occupant of a layer's row: a single BN node, ``"latent"`` or ``"variable"``."""

    kind: str
    node: Node


@dataclass
class _RelationshipGrouping:
    """
    Which real variables a class's 1-to-1 relationships flattened from a nested value
    object. These variables are excluded from their owner's own panel entirely — they
    get their own sibling panel instead, the same as an exchangeable child does.
    """

    prefix_of_variable_index: Dict[int, str]
    members_of_prefix: Dict[str, List[RealVariableNode]]
    label_of_prefix: Dict[str, str]
    key_of_prefix: Dict[str, str]


def _group_variables_by_single_relationship(
    rpc: RelationalProbabilisticCircuit, bn: BayesianNetwork, layer: Dict[int, int]
) -> _RelationshipGrouping:
    """
    Group every real variable of ``bn`` that a 1-to-1 relationship on ``rpc.class_``
    flattened from one nested value object (e.g. all of ``"SceneRoom.position.{x,y,z}"``
    from a ``position: KRROODPosition`` field), so its own panel can be excised from the
    owner's layout and rendered as a sibling panel instead.

    :param rpc: The relational circuit whose schema names the 1-to-1 relationships.
    :param bn: The Bayesian network reduced from ``rpc.class_probabilistic_circuit``.
    :param layer: Each node's layer, as returned by :func:`_assign_layers`. Unused by
        the grouping itself, kept for callers that need it alongside this result.
    :return: The grouping, empty if the class has no 1-to-1 relationships.
    """
    class_name = rpc.class_.__name__
    prefixes: List[Tuple[str, str, str]] = [
        (
            f"{class_name}.{relationship.key}.",
            relationship.domain_type.__name__,
            relationship.key,
        )
        for relationship in (
            rpc.schema_information.single_relationships if rpc.schema_information else ()
        )
    ]

    def matching_group(variable_name: str) -> Optional[Tuple[str, str, str]]:
        for prefix, label, key in prefixes:
            if variable_name.startswith(prefix):
                return prefix, label, key
        return None

    prefix_of_variable_index: Dict[int, str] = {}
    members_of_prefix: Dict[str, List[RealVariableNode]] = defaultdict(list)
    label_of_prefix: Dict[str, str] = {}
    key_of_prefix: Dict[str, str] = {}
    for node in bn.nodes():
        if isinstance(node, RealVariableNode):
            match = matching_group(node.variable.name)
            if match is not None:
                prefix, label, key = match
                prefix_of_variable_index[node.index] = prefix
                members_of_prefix[prefix].append(node)
                label_of_prefix[prefix] = label
                key_of_prefix[prefix] = key

    return _RelationshipGrouping(
        prefix_of_variable_index=prefix_of_variable_index,
        members_of_prefix=dict(members_of_prefix),
        label_of_prefix=label_of_prefix,
        key_of_prefix=key_of_prefix,
    )


def _render_value_class_panel(
    ax,
    class_label: str,
    subfield_labels: List[str],
    x_offset: float,
    y_offset: float,
    depth: int,
) -> _BNPanelResult:
    """
    Draw a plain panel for a 1-to-1 nested value class, e.g. the ``KRROODPosition``
    that a ``position`` field flattened into its owner's own circuit.

    Unlike :func:`_render_class_bn_panel`, there is no Bayesian network to reduce here
    — a value object like a position or orientation is never separately fit; it is only
    ever a handful of scalar columns living inside its owner's circuit. This still gets
    the same panel weight (border, header) as a real class panel, so a viewer reads it
    as its own encapsulated class rather than a special case.

    :param ax: The axes to draw on.
    :param class_label: The nested value class's name, for the panel header.
    :param subfield_labels: The bare subfield names (e.g. ``"x"``, ``"y"``, ``"z"``).
    :param x_offset: Left edge of this panel in data coordinates.
    :param y_offset: Top edge of this panel in data coordinates.
    :param depth: Current relational nesting depth, for its border color.
    :return: This panel's geometry, for the caller to stack sibling panels under.
    """
    class_color = CLASS_PALETTE[depth % len(CLASS_PALETTE)]
    count = len(subfield_labels)

    width = max(count * NODE_SPACING, MIN_PANEL_WIDTH) + 2 * PANEL_MARGIN
    height = PANEL_HEADER_HEIGHT + LEVEL_SPACING + PANEL_MARGIN

    ax.add_patch(
        FancyBboxPatch(
            (x_offset, y_offset), width, height,
            boxstyle="round,pad=0,rounding_size=0.08",
            facecolor=mcolors.to_rgba(class_color, alpha=0.05),
            edgecolor=class_color, linewidth=1.3,
        )
    )
    ax.text(
        x_offset + width / 2, y_offset + 0.2, class_label,
        ha="center", va="center", fontsize=10.5, fontweight="bold", color=class_color,
    )
    ax.text(
        x_offset + width / 2, y_offset + 0.47, f"{class_label} · {count} vars",
        ha="center", va="center", fontsize=7.5, color="0.45", family="monospace",
    )

    row_y = y_offset + PANEL_HEADER_HEIGHT + LEVEL_SPACING / 2
    for index, label in enumerate(sorted(subfield_labels)):
        x = x_offset + width / 2 + (index - (count - 1) / 2) * NODE_SPACING
        ax.add_patch(
            FancyBboxPatch(
                (x - VARIABLE_BOX_WIDTH / 2, row_y - VARIABLE_BOX_HEIGHT / 2),
                VARIABLE_BOX_WIDTH, VARIABLE_BOX_HEIGHT,
                boxstyle="round,pad=0,rounding_size=0.05",
                facecolor="white", edgecolor=class_color, linewidth=1.0, zorder=3,
            )
        )
        ax.text(
            x, row_y, label, ha="center", va="center",
            fontsize=6.6, family="monospace", zorder=3,
        )

    return _BNPanelResult(width=width, height=height)


def _render_class_bn_panel(
    ax,
    rpc: RelationalProbabilisticCircuit,
    class_label: str,
    x_offset: float,
    y_offset: float,
    depth: int,
    max_depth: int,
) -> _BNPanelResult:
    """
    Draw one class's Bayesian-network panel and recurse into its exchangeable child
    templates.

    :param ax: The axes to draw on.
    :param rpc: The relational circuit whose ``class_probabilistic_circuit`` is reduced
        for this panel, and whose exchangeable templates its children come from.
    :param class_label: Display name for the panel header.
    :param x_offset: Left edge of this panel in data coordinates.
    :param y_offset: Top edge of this panel in data coordinates.
    :param depth: Current relational nesting depth, for the recursion cutoff.
    :param max_depth: Maximum relational nesting depth to render.
    :return: This panel's geometry, for the caller to stack sibling panels under.
    """
    circuit = rpc.class_probabilistic_circuit
    bn = build_bayesian_network(circuit)
    layer = _assign_layers(bn)

    grouping = _group_variables_by_single_relationship(rpc, bn, layer)

    # Variables that a 1-to-1 relationship flattened from a nested value object are
    # excluded from this panel's own layout entirely — they get their own sibling
    # panel below, the same as an exchangeable child does, instead of a box nested
    # inside this one.
    slots_by_layer: Dict[int, List[_LayoutSlot]] = defaultdict(list)
    for node in bn.nodes():
        if isinstance(node, RealVariableNode) and node.index in grouping.prefix_of_variable_index:
            continue
        depth_index = layer[node.index]
        kind = "latent" if isinstance(node, LatentNode) else "variable"
        slots_by_layer[depth_index].append(_LayoutSlot(kind=kind, node=node))

    max_layer_count = max(len(slots) for slots in slots_by_layer.values())
    tree_depth = max(slots_by_layer)

    class_color = CLASS_PALETTE[depth % len(CLASS_PALETTE)]

    width = max(max_layer_count * NODE_SPACING, MIN_PANEL_WIDTH) + 2 * PANEL_MARGIN
    # +0.3 leaves room for a staggered aggregation-diamond label band below the
    # deepest layer, in case bridges land there.
    height = PANEL_HEADER_HEIGHT + (tree_depth + 1) * LEVEL_SPACING + PANEL_MARGIN + 0.55

    latent_to_field: Dict[Variable, str] = {
        latent: field_name
        for field_name, template in rpc.exchangeable_distribution_templates.items()
        for latent in template.latent_variables
    }

    def to_world(layer_index: int, position_in_layer: int, layer_count: int) -> Tuple[float, float]:
        local_x = (
            x_offset + width / 2 + (position_in_layer - (layer_count - 1) / 2) * NODE_SPACING
        )
        local_y = y_offset + PANEL_HEADER_HEIGHT + layer_index * LEVEL_SPACING
        return local_x, local_y

    ax.add_patch(
        FancyBboxPatch(
            (x_offset, y_offset),
            width,
            height,
            boxstyle="round,pad=0,rounding_size=0.08",
            facecolor=mcolors.to_rgba(class_color, alpha=0.05),
            edgecolor=class_color,
            linewidth=1.3,
        )
    )
    ax.text(
        x_offset + width / 2, y_offset + 0.2, class_label,
        ha="center", va="center", fontsize=10.5, fontweight="bold", color=class_color,
    )
    latent_total = sum(1 for node in bn.nodes() if isinstance(node, LatentNode))
    variable_total = len(bn.nodes()) - latent_total
    ax.text(
        x_offset + width / 2, y_offset + 0.47,
        f"BN({class_label}) · {variable_total} vars, {latent_total} latent",
        ha="center", va="center", fontsize=7.5, color="0.45", family="monospace",
    )

    positions: Dict[int, Tuple[float, float]] = {}
    for layer_index, slots in slots_by_layer.items():
        count = len(slots)
        for position_in_layer, slot in enumerate(slots):
            positions[slot.node.index] = to_world(layer_index, position_in_layer, count)

    # edges, drawn before nodes so glyphs sit on top. An edge whose child moved into a
    # 1-to-1 sibling panel is not drawn here at all — its source position is collected
    # instead, so the caller can bridge straight into that panel — and is collected once
    # per contributing latent even when several of its subfields share that latent.
    value_bridge_positions: Dict[str, List[Tuple[float, float]]] = defaultdict(list)
    value_bridge_seen = set()
    drawn_edges = set()
    for parent, child in bn.edges():
        if isinstance(child, RealVariableNode):
            prefix = grouping.prefix_of_variable_index.get(child.index)
            if prefix is not None:
                bridge_key = (prefix, parent.index)
                if bridge_key not in value_bridge_seen:
                    value_bridge_seen.add(bridge_key)
                    value_bridge_positions[prefix].append(positions[parent.index])
                continue
        edge_key = (parent.index, child.index)
        if edge_key in drawn_edges:
            continue
        drawn_edges.add(edge_key)
        x0, y0 = positions[parent.index]
        x1, y1 = positions[child.index]
        ax.add_patch(
            FancyArrowPatch(
                (x0, y0 + 0.06), (x1, y1 - 0.06),
                arrowstyle="-|>", mutation_scale=9,
                color=LATENT_COLOR, alpha=0.7, linewidth=1.0, zorder=1,
            )
        )

    latent_display_names: Dict[int, str] = {
        node.index: f"Z{sequence}"
        for sequence, node in enumerate(
            (node for node in bn.nodes() if isinstance(node, LatentNode)), start=1
        )
    }

    bridge_positions: Dict[str, List[Tuple[float, float]]] = {}
    # Adjacent aggregation diamonds keep their full name, which runs wide — alternate
    # their label band so two neighbors' labels stack diagonally instead of colliding.
    diamond_sequence = 0

    for slots in slots_by_layer.values():
        for slot in slots:
            if slot.kind == "latent":
                node = slot.node
                x, y = positions[node.index]
                ax.add_patch(
                    Circle(
                        (x, y), 0.11,
                        facecolor=mcolors.to_rgba(LATENT_COLOR, alpha=0.15),
                        edgecolor=LATENT_COLOR, linewidth=1.3, zorder=3,
                    )
                )
                ax.text(
                    x, y, f"{latent_display_names[node.index]}\nc={node.cardinality}",
                    ha="center", va="center", fontsize=5.6, family="monospace",
                    fontweight="bold", color=LATENT_COLOR, zorder=3,
                )
            else:
                node = slot.node
                x, y = positions[node.index]
                if node.variable in latent_to_field:
                    field_name = latent_to_field[node.variable]
                    wrapped = _wrap_label(node.variable.name, max_chars=16)
                    size = 0.11
                    ax.add_patch(
                        Polygon(
                            [(x, y - size), (x + size, y), (x, y + size), (x - size, y)],
                            closed=True, facecolor="white", edgecolor=LATENT_COLOR,
                            linewidth=1.4, zorder=3,
                        )
                    )
                    label_y = y + (0.2, 0.44, 0.68)[diamond_sequence % 3]
                    diamond_sequence += 1
                    ax.text(
                        x, label_y, wrapped, ha="center", va="center",
                        fontsize=5.8 if "\n" in wrapped else 6.6,
                        family="monospace", color=LATENT_COLOR, zorder=3,
                    )
                    bridge_positions.setdefault(field_name, []).append((x, y))
                else:
                    label = _wrap_label(_short_label(node.variable.name))
                    is_wrapped = "\n" in label
                    box_h = VARIABLE_BOX_HEIGHT * 1.4 if is_wrapped else VARIABLE_BOX_HEIGHT
                    ax.add_patch(
                        FancyBboxPatch(
                            (x - VARIABLE_BOX_WIDTH / 2, y - box_h / 2),
                            VARIABLE_BOX_WIDTH, box_h,
                            boxstyle="round,pad=0,rounding_size=0.05",
                            facecolor="white", edgecolor=class_color, linewidth=1.0, zorder=3,
                        )
                    )
                    ax.text(
                        x, y, label, ha="center", va="center",
                        fontsize=5.6 if is_wrapped else 6.6, family="monospace", zorder=3,
                    )

    if depth < max_depth:
        child_x = x_offset + width + PANEL_GAP
        label_x = x_offset + width + PANEL_GAP / 2
        child_y = y_offset

        def bridge_into_child(
            sources: List[Tuple[float, float]],
            child_result: _BNPanelResult,
            label: str,
            dashed: bool,
        ) -> None:
            nonlocal child_y
            entry_top = child_y + PANEL_HEADER_HEIGHT + 0.15
            # Sorting by source height keeps the elbow bends stacked in the same order
            # they leave the source panel, so they run in clean parallel bands instead
            # of crossing each other on the way to the child.
            for index, (source_x, source_y) in enumerate(
                sorted(sources, key=lambda point: point[1])
            ):
                target_y = entry_top + index * 0.22
                # A manually built elbow (source -> mid -> mid -> child) instead of a
                # connectionstyle: it leaves the source horizontally, drops straight
                # down/up to the target's height, then arrives horizontally — and,
                # being three explicit line segments, never hits the "parallel tangent"
                # degenerate case an automatic angle connector can when a source and
                # target land at the same height.
                elbow = Path(
                    [
                        (source_x, source_y),
                        (label_x, source_y),
                        (label_x, target_y),
                        (child_x, target_y),
                    ],
                    [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO],
                )
                ax.add_patch(
                    FancyArrowPatch(
                        path=elbow, arrowstyle="-|>", mutation_scale=9,
                        linestyle=(0, (4, 3)) if dashed else "solid",
                        color=LATENT_COLOR, linewidth=1.1, zorder=2,
                    )
                )
            ax.text(
                label_x, child_y + PANEL_HEADER_HEIGHT, label,
                ha="center", va="center", fontsize=6.8, family="monospace",
                color=LATENT_COLOR,
            )
            child_y = child_y + child_result.height + PANEL_GAP * 0.4

        # Exchangeable (1-to-many) children: dashed bridges, "×N" — several instances
        # ground under this class per parent.
        for field_name in bridge_positions:
            template = rpc.exchangeable_distribution_templates[field_name]
            child_class_name = template.template_distribution.class_.__name__.removesuffix(
                "DAO"
            )
            child_result = _render_class_bn_panel(
                ax, template.template_distribution, child_class_name,
                child_x, child_y, depth + 1, max_depth,
            )
            bridge_into_child(
                bridge_positions[field_name], child_result, f"{field_name}\n×N", dashed=True
            )

        # 1-to-1 nested value classes: solid bridges, no "×N" — exactly one instance
        # per parent, and no Bayesian-network structure of its own to reduce.
        for prefix, sources in value_bridge_positions.items():
            subfield_labels = [
                node.variable.name[len(prefix) :] for node in grouping.members_of_prefix[prefix]
            ]
            child_result = _render_value_class_panel(
                ax, grouping.label_of_prefix[prefix], subfield_labels,
                child_x, child_y, depth + 1,
            )
            bridge_into_child(
                sources, child_result, grouping.key_of_prefix[prefix], dashed=False
            )

    return _BNPanelResult(width=width, height=height)


def plot_relational_bayesian_network(
    rpc: RelationalProbabilisticCircuit, max_depth: int = 3
) -> Figure:
    """
    Plot a :class:`RelationalProbabilisticCircuit` as Bayesian-network panels,
    Proposal-E style: one bordered panel per class in the relational structure, each
    holding that class's own circuit reduced to its induced Bayesian network. An
    aggregation statistic — a real variable that also serves as an exchangeable child
    template's latent variable — is drawn as a diamond, the same convention the vtree
    view uses, with edges crossing into that child's own panel, recursed up to
    ``max_depth`` relational hops.

    :param rpc: A fitted relational circuit (``class_probabilistic_circuit`` must not
        be ``None``).
    :param max_depth: How many levels of exchangeable relations to recurse into.
    :return: The matplotlib figure. Caller owns saving/closing it.
    """
    if rpc.class_probabilistic_circuit is None:
        raise ValueError("rpc must be fitted before it can be plotted.")

    fig, ax = plt.subplots()
    _render_class_bn_panel(
        ax, rpc, rpc.class_.__name__, x_offset=0.0, y_offset=0.0, depth=0, max_depth=max_depth
    )
    ax.relim()
    x_min, x_max = ax.dataLim.xmin, ax.dataLim.xmax
    y_min, y_max = ax.dataLim.ymin, ax.dataLim.ymax
    margin = 0.3
    ax.set_xlim(x_min - margin, x_max + margin)
    ax.set_ylim(y_max + margin, y_min - margin)
    ax.axis("off")

    width_units = (x_max - x_min) + 2 * margin
    height_units = (y_max - y_min) + 2 * margin
    fig.set_size_inches(
        min(max(width_units * INCHES_PER_UNIT_X, 6.0), 26.0),
        min(max(height_units * INCHES_PER_UNIT_Y, 3.5), 16.0),
    )
    fig.tight_layout()
    return fig

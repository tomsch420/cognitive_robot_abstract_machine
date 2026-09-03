"""
Variable-tree ("vtree") visualization for :class:`RelationalProbabilisticCircuit`.

Renders one bordered panel per class: a recursive binary partition of that class's own
circuit variables, with each split annotated by how many fitted circuit nodes collapse
under it. Panel size is bounded by variable count rather than node count, so it stays
small even when ``class_probabilistic_circuit`` itself runs to thousands of nodes.
Latent variables that bridge into an exchangeable child's template are drawn as diamonds
whose edges cross into that child class's own panel, recursively.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch, Polygon
from random_events.variable import Variable
from typing_extensions import Dict, List, Sequence, Tuple

from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)
from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    ProbabilisticCircuit,
)

LEAF_SPACING = 1.55
LEVEL_SPACING = 0.85
PANEL_HEADER_HEIGHT = 0.85
PANEL_MARGIN = 0.4
PANEL_GAP = 1.3
MIN_PANEL_WIDTH = 1.6
LEAF_BOX_WIDTH = LEAF_SPACING * 0.8
LEAF_BOX_HEIGHT = 0.24
HOT_COMPLEXITY_FRACTION = 0.5
"""A split is drawn as an accent-colored hotspot once its complexity reaches this
fraction of its panel root's complexity."""
INCHES_PER_UNIT_X = 0.72
INCHES_PER_UNIT_Y = 0.95
"""Figure size scales with layout extent at this ratio, so a fixed point size never
has to fit into a box that shrinks as more variables are added."""


@dataclass
class VTreeNode:
    """One node of a variable tree: a recursive partition of a scope."""

    variables: List[Variable]
    children: List["VTreeNode"] = field(default_factory=list)
    complexity: int = 0
    """Count of fitted circuit nodes whose own scope collapses under this node."""

    @property
    def is_leaf(self) -> bool:
        return not self.children


def build_vtree(variables: Sequence[Variable]) -> VTreeNode:
    """
    Recursively partition ``variables`` into a binary vtree.

    Splits a name-sorted scope in half at every level until each leaf holds exactly one
    variable. The split is deterministic and name-based, not correlation-based — it
    bounds panel size by variable count, and makes no claim about which variables
    actually interact in the fit.
    """
    return _build_vtree(sorted(variables, key=lambda variable: variable.name))


def _build_vtree(variables: List[Variable]) -> VTreeNode:
    if len(variables) <= 1:
        return VTreeNode(variables=list(variables))
    midpoint = len(variables) // 2
    return VTreeNode(
        variables=list(variables),
        children=[_build_vtree(variables[:midpoint]), _build_vtree(variables[midpoint:])],
    )


def annotate_complexity(vtree: VTreeNode, circuit: ProbabilisticCircuit) -> None:
    """
    Annotate every node of ``vtree`` with how much of ``circuit`` lives under its scope.

    A node's complexity is the number of circuit nodes (sum, product, or leaf) whose own
    variable scope is entirely contained in the vtree node's scope — the count of fitted
    components a viewer would otherwise have to look past to see only that subtree.

    :param vtree: The vtree to annotate in place.
    :param circuit: The fitted circuit whose nodes are counted per scope.
    """
    scoped_nodes = [
        set(node.variables) for node in circuit.nodes() if len(node.variables) > 0
    ]
    _annotate(vtree, scoped_nodes)


def _annotate(node: VTreeNode, scoped_nodes: List[set]) -> None:
    scope = set(node.variables)
    node.complexity = sum(1 for node_scope in scoped_nodes if node_scope <= scope)
    for child in node.children:
        _annotate(child, scoped_nodes)


def _short_label(variable_name: str) -> str:
    """Drop a leading ``ClassName.`` namespace segment, if the name has one."""
    return variable_name.split(".", 1)[1] if "." in variable_name else variable_name


def _wrap_label(label: str, max_chars: int = 14) -> str:
    """Break a long label onto two lines at the ``.``/``_`` nearest its midpoint."""
    if len(label) <= max_chars:
        return label
    split_points = [index for index, char in enumerate(label) if char in "._"]
    if split_points:
        midpoint = len(label) / 2
        split_at = min(split_points, key=lambda index: abs(index - midpoint))
        return label[: split_at + 1] + "\n" + label[split_at + 1 :]
    half = len(label) // 2
    return label[:half] + "\n" + label[half:]


@dataclass
class _LayoutNode:
    node: VTreeNode
    x: float
    y: int


def _layout(vtree: VTreeNode) -> Tuple[List[_LayoutNode], Dict[int, List[_LayoutNode]], int]:
    """
    Assign a leaf-count-based x position and a depth-based y position to every node.

    :return: A flat list of positioned nodes, the same nodes grouped by depth, and the
        tree's leaf count (used to size its panel).
    """
    leaves: List[VTreeNode] = []

    def collect_leaves(current: VTreeNode) -> None:
        if current.is_leaf:
            leaves.append(current)
        for child in current.children:
            collect_leaves(child)

    collect_leaves(vtree)
    leaf_x = {id(leaf): index for index, leaf in enumerate(leaves)}

    positioned: List[_LayoutNode] = []
    by_depth: Dict[int, List[_LayoutNode]] = {}

    def assign(current: VTreeNode, depth: int) -> float:
        if current.is_leaf:
            x = float(leaf_x[id(current)])
        else:
            x = sum(assign(child, depth + 1) for child in current.children) / len(
                current.children
            )
        entry = _LayoutNode(node=current, x=x, y=depth)
        positioned.append(entry)
        by_depth.setdefault(depth, []).append(entry)
        return x

    assign(vtree, 0)
    return positioned, by_depth, max(len(leaves), 1)


@dataclass
class _PanelResult:
    width: float
    height: float
    root_anchor: Tuple[float, float]
    bridge_positions: Dict[str, List[Tuple[float, float]]]


def _render_class_panel(
    ax,
    rpc: RelationalProbabilisticCircuit,
    class_label: str,
    x_offset: float,
    y_offset: float,
    depth: int,
    max_depth: int,
) -> _PanelResult:
    """
    Draw one class's vtree panel and recurse into its exchangeable child templates.

    :param ax: The axes to draw on.
    :param rpc: The relational circuit whose ``class_probabilistic_circuit`` and
        exchangeable templates this panel and its children come from.
    :param class_label: Display name for the panel header.
    :param x_offset: Left edge of this panel in data coordinates.
    :param y_offset: Top edge of this panel in data coordinates.
    :param depth: Current relational nesting depth, for the recursion cutoff.
    :param max_depth: Maximum relational nesting depth to render.
    :return: This panel's geometry, for the caller to route incoming bridge edges to.
    """
    circuit = rpc.class_probabilistic_circuit
    vtree = build_vtree(circuit.variables)
    annotate_complexity(vtree, circuit)
    positioned, by_depth, leaf_count = _layout(vtree)
    tree_depth = max(entry.y for entry in positioned)

    width = max(leaf_count * LEAF_SPACING, MIN_PANEL_WIDTH) + 2 * PANEL_MARGIN
    height = (
        PANEL_HEADER_HEIGHT + (tree_depth + 1) * LEVEL_SPACING + PANEL_MARGIN
    )

    latent_to_field: Dict[Variable, str] = {
        latent: field_name
        for field_name, template in rpc.exchangeable_distribution_templates.items()
        for latent in template.latent_variables
    }

    def to_world(x: float, y: int) -> Tuple[float, float]:
        inner_width = width - 2 * PANEL_MARGIN
        local_x = (
            x_offset + PANEL_MARGIN + inner_width * 0.5
            if leaf_count == 1
            else x_offset + PANEL_MARGIN + (x / (leaf_count - 1)) * inner_width
        )
        local_y = y_offset + PANEL_HEADER_HEIGHT + y * LEVEL_SPACING
        return local_x, local_y

    ax.add_patch(
        FancyBboxPatch(
            (x_offset, y_offset),
            width,
            height,
            boxstyle="round,pad=0,rounding_size=0.08",
            facecolor="none",
            edgecolor="0.25",
            linewidth=1.1,
        )
    )
    ax.text(
        x_offset + width / 2,
        y_offset + 0.2,
        class_label,
        ha="center",
        va="center",
        fontsize=10.5,
        fontweight="bold",
    )
    ax.text(
        x_offset + width / 2,
        y_offset + 0.47,
        "vtree(" + class_label + f") · {len(circuit.variables)} vars",
        ha="center",
        va="center",
        fontsize=7.5,
        color="0.45",
        family="monospace",
    )

    positions = {id(entry.node): to_world(entry.x, entry.y) for entry in positioned}

    # edges, drawn before nodes so glyphs sit on top
    def draw_edges(current: VTreeNode) -> None:
        for child in current.children:
            x0, y0 = positions[id(current)]
            x1, y1 = positions[id(child)]
            ax.plot([x0, x1], [y0 + 0.06, y1 - 0.06], color="0.55", linewidth=0.9, zorder=1)
            draw_edges(child)

    draw_edges(vtree)

    root_complexity = vtree.complexity
    bridge_positions: Dict[str, List[Tuple[float, float]]] = {}

    for entry in positioned:
        x, y = positions[id(entry.node)]
        if entry.node.is_leaf:
            variable = entry.node.variables[0]
            label = _short_label(variable.name)
            if variable in latent_to_field:
                field_name = latent_to_field[variable]
                size = 0.11
                ax.add_patch(
                    Polygon(
                        [(x, y - size), (x + size, y), (x, y + size), (x - size, y)],
                        closed=True,
                        facecolor="white",
                        edgecolor="#bf6a2e",
                        linewidth=1.4,
                        zorder=3,
                    )
                )
                ax.text(
                    x, y + 0.2, label, ha="center", va="center", fontsize=6.6,
                    family="monospace", color="0.35", zorder=3,
                )
                bridge_positions.setdefault(field_name, []).append((x, y))
            else:
                wrapped = _wrap_label(label)
                is_wrapped = "\n" in wrapped
                box_h = LEAF_BOX_HEIGHT * 1.7 if is_wrapped else LEAF_BOX_HEIGHT
                ax.add_patch(
                    FancyBboxPatch(
                        (x - LEAF_BOX_WIDTH / 2, y - box_h / 2),
                        LEAF_BOX_WIDTH,
                        box_h,
                        boxstyle="round,pad=0,rounding_size=0.05",
                        facecolor="white",
                        edgecolor="0.25",
                        linewidth=1.0,
                        zorder=3,
                    )
                )
                ax.text(
                    x, y, wrapped, ha="center", va="center",
                    fontsize=5.6 if is_wrapped else 6.6,
                    family="monospace", zorder=3,
                )
        else:
            is_hot = (
                root_complexity > 0
                and entry.node.complexity >= HOT_COMPLEXITY_FRACTION * root_complexity
            )
            ax.add_patch(
                Circle(
                    (x, y), 0.075,
                    facecolor="white",
                    edgecolor="#bf6a2e" if is_hot else "0.25",
                    linewidth=1.3,
                    zorder=3,
                )
            )
            ax.text(
                x, y - 0.16, f"k = {entry.node.complexity:,}",
                ha="center", va="center", fontsize=6.8, family="monospace",
                color="#bf6a2e" if is_hot else "0.5", zorder=3,
            )

    root_anchor = positions[id(vtree)]

    if depth < max_depth:
        child_x = x_offset + width + PANEL_GAP
        child_y = y_offset
        for field_name in bridge_positions:
            template = rpc.exchangeable_distribution_templates[field_name]
            child_class_name = template.template_distribution.class_.__name__.removesuffix(
                "DAO"
            )
            child_result = _render_class_panel(
                ax,
                template.template_distribution,
                child_class_name,
                child_x,
                child_y,
                depth + 1,
                max_depth,
            )
            sources = bridge_positions[field_name]
            entry_top = child_y + PANEL_HEADER_HEIGHT + 0.15
            for index, (source_x, source_y) in enumerate(sources):
                target_y = entry_top + index * 0.22
                ax.add_patch(
                    FancyArrowPatch(
                        (source_x, source_y),
                        (child_x, target_y),
                        connectionstyle="arc3,rad=0.12",
                        arrowstyle="-|>",
                        mutation_scale=8,
                        linestyle=(0, (4, 3)),
                        color="#bf6a2e",
                        linewidth=1.0,
                        zorder=2,
                    )
                )
            label_x = x_offset + width + PANEL_GAP / 2
            label_y = y_offset + PANEL_HEADER_HEIGHT
            ax.text(
                label_x, label_y, f"{field_name}\n×N",
                ha="center", va="center", fontsize=6.8, family="monospace",
                color="#bf6a2e",
            )
            child_y = child_y + child_result.height + PANEL_GAP * 0.6

    return _PanelResult(
        width=width, height=height, root_anchor=root_anchor, bridge_positions=bridge_positions
    )


def plot_relational_vtree(
    rpc: RelationalProbabilisticCircuit, max_depth: int = 3
) -> Figure:
    """
    Plot a :class:`RelationalProbabilisticCircuit` as vtree panels, Proposal-E style.

    One bordered panel per class in the relational structure, each holding a variable
    tree (not a circuit) over that class's own scope, annotated with how many fitted
    circuit nodes collapse under each split. Latent variables that bridge into an
    exchangeable child template are drawn as diamonds with edges crossing into that
    child's own panel, recursed up to ``max_depth`` relational hops.

    :param rpc: A fitted relational circuit (``class_probabilistic_circuit`` must not
        be ``None``).
    :param max_depth: How many levels of exchangeable relations to recurse into.
    :return: The matplotlib figure. Caller owns saving/closing it.
    """
    if rpc.class_probabilistic_circuit is None:
        raise ValueError("rpc must be fitted before it can be plotted.")

    fig, ax = plt.subplots()
    _render_class_panel(
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

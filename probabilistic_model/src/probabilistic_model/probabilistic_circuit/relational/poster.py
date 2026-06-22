"""
UML-style class diagram for :class:`RelationalProbabilisticCircuit`.

Visual language is inspired by the giskard motion-statechart plotter
(thick borders, coloured info bands, structured HTML-table compartments).

Node kinds:

* **Class node** (``<<circuit>>``) — header + alternating-row attribute
  table + three-cell stat band (sum nodes | product nodes | leaf nodes).
* **Sub-object node** — one per nested value-object group (e.g.
  ``orientation``), connected to its class by a composition arrow.
* **Template node** (``<<template>>``) — one per exchangeable-distribution
  template; sits between parent and child class, listing the latent
  conditioning variables.
* **Enum node** (``<<enumeration>>``) — one per unique enum type, placed to
  the right of its referencing class and linked by a dashed use-dependency.

Output: a single SVG file (or PNG / PDF).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Union

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import numpy as np

matplotlib.use("Agg")

from random_events.variable import Continuous, Symbolic, Integer

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    LeafUnit,
    ProductUnit,
    SumUnit,
)

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
    )


# ── palette ───────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class _NodeStyle:
    """Colour scheme for one node kind."""

    header: str
    header_fg: str
    row_odd: str
    row_even: str
    border: str
    stereo_fg: str = "#FFFFFF"
    legend_label: str = ""


_SHARED_ROW_ODD: str = "#EBF5FB"
_SHARED_ROW_EVEN: str = "#D6EAF8"

_STYLE_CLASS = _NodeStyle(
    header="#1B2631",
    header_fg="#FFFFFF",
    row_odd=_SHARED_ROW_ODD,
    row_even=_SHARED_ROW_EVEN,
    border="#1B2631",
    stereo_fg="#85C1E9",
    legend_label="<<circuit>>  class  (name + stats)",
)
_STYLE_ATTR = _NodeStyle(
    header="#1A5276",
    header_fg="#FFFFFF",
    row_odd=_SHARED_ROW_ODD,
    row_even=_SHARED_ROW_EVEN,
    border="#1A5276",
    legend_label="attribute group / sub-object",
)
_STYLE_EDT = _NodeStyle(
    header="#6E2F1A",
    header_fg="#FFFFFF",
    row_odd="#FDF2E9",
    row_even="#FAE5D3",
    border="#6E2F1A",
    stereo_fg="#F0B27A",
    legend_label="<<template>>  EDT  (latent conditioning vars)",
)
_STYLE_ENUM = _NodeStyle(
    header="#4A235A",
    header_fg="#FFFFFF",
    row_odd="#F5EEF8",
    row_even="#EAD9F4",
    border="#4A235A",
    stereo_fg="#D7BDE2",
    legend_label="<<enumeration>>  enum type",
)

# Stat-band colours (class node only) — sum (green) | product (blue) | leaf (purple)
_CC_STAT_SUM: str = "#1A9641"
_CC_STAT_PROD: str = "#2B83BA"
_CC_STAT_LEAF: str = "#6C3483"
_CC_STAT_FG: str = "#FFFFFF"

# Arrow colours
_CARR_COMP: str = "#1B2631"   # composition (class → attr node)
_CARR_EDT: str = "#6E2F1A"    # class → template and template → child class
_CARR_ENUM: str = "#7B2D8B"   # dashed use-dependency to enum
_CMULT: str = "#C0392B"

_TYPE_COLOR: dict[str, str] = {
    "Continuous": "#1A9641",
    "Integer": "#2B83BA",
    "Symbolic": "#7B2D8B",
}
_TYPE_SHORT: dict[str, str] = {
    "Continuous": "float",
    "Integer": "int",
}

# ── geometry ──────────────────────────────────────────────────────────────────

_CLASS_W: float = 4.4
_ATTR_W: float = 3.6
_EDT_W: float = 4.2
_ENUM_W: float = 3.2

_LH: float = 0.29            # row height
_HEADER_H: float = 0.76
_STAT_BAR_H: float = 0.40
_ROW_PAD_V: float = 0.10     # vertical padding above/below row block
_BW: float = 2.8             # border line-width for class nodes
_THIN_BW: float = 1.6        # border line-width for attr and EDT nodes

_COL_GAP: float = 1.0
_ROW_GAP: float = 2.0
_SUBOBJ_GAP: float = 1.2     # vertical gap between class box and its sub-object row
_ENUM_GAP: float = 1.4       # horizontal gap between the tree right edge and the enum column


# ── data model ────────────────────────────────────────────────────────────────


@dataclass
class _Attribute:
    name: str
    type_name: str
    enum_type_name: str = ""
    """For Symbolic variables, the name of the referenced enum type (e.g. ``SceneObjectType``)."""


@dataclass
class _CircuitStats:
    """Node-count breakdown of a probabilistic circuit DAG."""

    num_sum: int
    """Number of :class:`SumUnit` nodes (mixture components)."""

    num_product: int
    """Number of :class:`ProductUnit` nodes (independence factorisations)."""

    num_leaf: int
    """Number of leaf distribution nodes."""

    num_parameters: int
    """Total learnable parameters (sum weights + leaf distribution params)."""


@dataclass
class _EnumNode:
    """
    A standalone UML ``<<enumeration>>`` box for one enum type.

    Placed to the right of its primary referencing class node and linked
    by a dashed use-dependency arrow.
    """

    type_name: str
    """Python enum class name (e.g. ``SceneObjectType``)."""

    values: list[str]
    """Enum member names in declaration order."""

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_ENUM_W, init=False)
    height: float = field(default=0.0, init=False)


@dataclass
class _AttrNode:
    """
    A sub-object association box for one nested value-object group.

    For example, variables ``SceneRoom.orientation.{w,x,y,z}`` produce one
    :class:`_AttrNode` labelled ``orientation``.
    """

    label: str
    """Sub-object group name (e.g. ``orientation``)."""

    attributes: list[_Attribute]

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_ATTR_W, init=False)
    height: float = field(default=0.0, init=False)


@dataclass
class _EDTNode:
    """An exchangeable-distribution template box sitting between parent and child."""

    relation_name: str
    latent_attributes: list[_Attribute]
    child: _ClassNode

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_EDT_W, init=False)
    height: float = field(default=0.0, init=False)


@dataclass
class _ClassNode:
    """
    A circuit-class box.

    Shows the class header, all modelled scalar attributes in an
    alternating-row table, and a three-cell stat band (sum nodes |
    product nodes | leaf nodes).  Sub-object groups and EDT children are
    rendered as separate connected nodes.
    """

    class_name: str
    stats: _CircuitStats
    """Circuit node-count breakdown and parameter count."""

    attributes: list[_Attribute]
    """All scalar attributes shown inside the class box."""

    sub_object_nodes: list[_AttrNode]
    """One association box per sub-object group (e.g. orientation, position)."""

    edt_children: list[_EDTNode]

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_CLASS_W, init=False)
    height: float = field(default=0.0, init=False)


# _EnumNode is excluded from _AnyNode because it lives outside the tree and is
# managed separately in _build (placed beside its referencing class).
_AnyNode = Union[_ClassNode, _AttrNode, _EDTNode]


# ── extraction ────────────────────────────────────────────────────────────────


def _var_type(variable) -> str:
    if isinstance(variable, Continuous):
        return "Continuous"
    if isinstance(variable, Symbolic):
        return "Symbolic"
    if isinstance(variable, Integer):
        return "Integer"
    return type(variable).__name__


def _strip_prefix(name: str, class_name: str) -> str:
    for prefix in (f"{class_name}Aggregations.", f"{class_name}."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _compute_stats(rspn: RelationalProbabilisticCircuit) -> _CircuitStats:
    """
    Compute the node-count breakdown and parameter count for a class circuit.

    Learnable parameters are: one free mixture weight per sum-node child
    minus one (since weights are normalised), plus the parameters of each
    leaf distribution.  :class:`DiracDelta` distributions have no free
    parameters and are excluded from the count.

    :param rspn: Fitted RSPN whose class circuit to inspect.
    :return: :class:`_CircuitStats` with counts and parameter total.
    """
    pc = rspn.class_probabilistic_circuit
    num_sum = num_product = num_leaf = num_parameters = 0

    for node in pc.nodes():
        if isinstance(node, SumUnit):
            num_sum += 1
            k = sum(1 for _ in pc.graph.successors(node.index))
            num_parameters += max(k - 1, 0)
        elif isinstance(node, ProductUnit):
            num_product += 1
        elif isinstance(node, LeafUnit):
            num_leaf += 1

    for leaf in pc.leaves:
        d = leaf.distribution
        if hasattr(d, "probabilities"):
            num_parameters += len(d.probabilities)
        elif not hasattr(d, "location"):
            # Non-Dirac continuous distributions without a probabilities table
            num_parameters += 1

    return _CircuitStats(
        num_sum=num_sum,
        num_product=num_product,
        num_leaf=num_leaf,
        num_parameters=num_parameters,
    )


def _build_tree(
    rspn: RelationalProbabilisticCircuit,
    latent_names: set[str],
    enum_registry: dict[str, _EnumNode] | None = None,
) -> _ClassNode:
    """
    Recursively extract the diagram node tree from a fitted RSPN.

    Scalar variables with a sub-object prefix (e.g. ``orientation.w``) are
    separated into :class:`_AttrNode` sub-object boxes; the remaining flat
    attributes are listed directly in the class box.  Latent conditioning
    variables and aggregation statistics are excluded.

    :param rspn: The fitted RSPN to visualise.
    :param latent_names: Variable names to suppress in this class.
    :param enum_registry: Shared dict accumulating unique :class:`_EnumNode`
        instances keyed by enum type name.  Passing the same dict for all
        recursive calls ensures each enum type produces exactly one box.
    :return: Root :class:`_ClassNode` of the diagram tree.
    """
    if enum_registry is None:
        enum_registry = {}

    pc = rspn.class_probabilistic_circuit
    class_name = rspn.class_.__name__

    agg_names: set[str] = set()
    if rspn.feature_extractor is not None:
        for variables in rspn.feature_extractor.exchangeable_features.values():
            agg_names.update(v._name_ for v in variables)

    sub_groups: dict[str, list[_Attribute]] = {}
    class_box_attrs: list[_Attribute] = []

    for var in pc.variables:
        if var.name in latent_names or var.name in agg_names:
            continue
        display = _strip_prefix(var.name, class_name)
        enum_type_name = ""
        if isinstance(var, Symbolic):
            members = list(var.domain)
            raw_type = type(members[0]).__name__ if members else "Enum"
            enum_type_name = raw_type
            if raw_type not in enum_registry:
                enum_registry[raw_type] = _EnumNode(
                    type_name=raw_type,
                    values=[str(v).split(".")[-1] for v in members],
                )
        attr = _Attribute(name=display, type_name=_var_type(var), enum_type_name=enum_type_name)
        parts = display.split(".")
        if len(parts) >= 2:
            sub_groups.setdefault(parts[0], []).append(
                _Attribute(name=".".join(parts[1:]), type_name=attr.type_name,
                           enum_type_name=enum_type_name)
            )
        else:
            class_box_attrs.append(attr)

    sub_object_nodes = [
        _AttrNode(label=name, attributes=attrs)
        for name, attrs in sub_groups.items()
    ]

    edt_children: list[_EDTNode] = []
    for rel_name, template in rspn.exchangeable_distribution_templates.items():
        child_latent_names = {v.name for v in template.latent_variables}
        latent_attrs = [
            _Attribute(
                name=_strip_prefix(v.name, class_name),
                type_name=_var_type(v),
            )
            for v in template.latent_variables
        ]
        child_node = _build_tree(template.template_distribution, child_latent_names,
                                 enum_registry)
        edt_children.append(
            _EDTNode(
                relation_name=rel_name,
                latent_attributes=latent_attrs,
                child=child_node,
            )
        )

    return _ClassNode(
        class_name=class_name,
        stats=_compute_stats(rspn),
        attributes=class_box_attrs,
        sub_object_nodes=sub_object_nodes,
        edt_children=edt_children,
    )


# ── height calculation ────────────────────────────────────────────────────────


def _rows_height(row_count: int) -> float:
    return row_count * _LH


def _boxed_rows_height(row_count: int) -> float:
    """Header + vertical padding + rows + vertical padding."""
    return _HEADER_H + _ROW_PAD_V + _rows_height(row_count) + _ROW_PAD_V


def _attr_node_height(n: _AttrNode) -> float:
    return _boxed_rows_height(len(n.attributes))


def _enum_node_height(n: _EnumNode) -> float:
    return _boxed_rows_height(len(n.values))


def _edt_node_height(n: _EDTNode) -> float:
    return _boxed_rows_height(max(len(n.latent_attributes), 1))


def _class_node_height(n: _ClassNode) -> float:
    inner = (_ROW_PAD_V + _rows_height(len(n.attributes)) + _ROW_PAD_V
             if n.attributes else 0.0)
    return _HEADER_H + inner + _STAT_BAR_H


def _assign_heights(node: _AnyNode) -> None:
    match node:
        case _AttrNode():
            node.height = _attr_node_height(node)
        case _EDTNode():
            node.height = _edt_node_height(node)
            _assign_heights(node.child)
        case _ClassNode():
            node.height = _class_node_height(node)
            for a in node.sub_object_nodes:
                _assign_heights(a)
            for e in node.edt_children:
                _assign_heights(e)
        case _:
            raise TypeError(f"Unexpected node type: {type(node)}")


# ── layout ────────────────────────────────────────────────────────────────────


def _row_total_w(items: list[_AnyNode], widths: list[float]) -> float:
    if not items:
        return 0.0
    return sum(widths) + _COL_GAP * (len(items) - 1)


def _subtree_w(node: _AnyNode) -> float:
    match node:
        case _AttrNode():
            return node.width
        case _EDTNode():
            return max(node.width, _subtree_w(node.child))
        case _ClassNode():
            edt_widths = [_subtree_w(e) for e in node.edt_children]
            sub_widths = [s.width for s in node.sub_object_nodes]
            edt_row_w = _row_total_w(node.edt_children, edt_widths)
            sub_row_w = _row_total_w(node.sub_object_nodes, sub_widths)
            return max(node.width, edt_row_w, sub_row_w)
        case _:
            raise TypeError(f"Unexpected node type: {type(node)}")


def _place_row(items: list[_AnyNode], widths: list[float],
               x_ctr: float, y_top: float) -> None:
    """Centre a list of items at *x_ctr* on the same horizontal band."""
    total_w = _row_total_w(items, widths)
    cursor = x_ctr - total_w / 2
    for item, w in zip(items, widths):
        _place(item, cursor + w / 2, y_top)
        cursor += w + _COL_GAP


def _place(node: _AnyNode, x_ctr: float, y_top: float) -> None:
    """Recursively assign positions; ``y_top`` is the top edge of *node*."""
    match node:
        case _AttrNode():
            node.x = x_ctr - node.width / 2
            node.y = y_top - node.height

        case _EDTNode():
            node.x = x_ctr - node.width / 2
            node.y = y_top - node.height
            _place(node.child, x_ctr, node.y - _ROW_GAP)

        case _ClassNode():
            node.x = x_ctr - node.width / 2
            node.y = y_top - node.height

            # Sub-object nodes occupy a dedicated row just below the class box.
            # EDT children go one further row below the sub-object row.
            cursor_y = node.y - _SUBOBJ_GAP
            if node.sub_object_nodes:
                sub_widths = [s.width for s in node.sub_object_nodes]
                _place_row(node.sub_object_nodes, sub_widths, x_ctr, cursor_y)
                tallest_sub = max(s.height for s in node.sub_object_nodes)
                cursor_y -= tallest_sub + _ROW_GAP

            if node.edt_children:
                edt_widths = [_subtree_w(e) for e in node.edt_children]
                _place_row(node.edt_children, edt_widths, x_ctr, cursor_y)

        case _:
            raise TypeError(f"Unexpected node type: {type(node)}")


def _collect(node: _AnyNode) -> list[_AnyNode]:
    result: list[_AnyNode] = [node]
    match node:
        case _AttrNode():
            pass
        case _EDTNode():
            result += _collect(node.child)
        case _ClassNode():
            for a in node.sub_object_nodes:
                result += _collect(a)
            for e in node.edt_children:
                result += _collect(e)
        case _:
            raise TypeError(f"Unexpected node type: {type(node)}")
    return result


def _bounds(nodes: list) -> tuple[float, float, float, float]:
    return (
        min(n.x for n in nodes),
        min(n.y for n in nodes),
        max(n.x + n.width for n in nodes),
        max(n.y + n.height for n in nodes),
    )


# ── enum placement ────────────────────────────────────────────────────────────


def _first_reference_depth(
    name: str,
    tree_nodes: list[_AnyNode],
) -> float:
    """Return the highest (most-positive) y top-edge among nodes referencing *name*.

    This sorts enum boxes so that those referenced by higher (earlier) levels
    appear at the top of the enum column.
    """
    tops = [n.y + n.height for n in tree_nodes if name in _enum_refs(n)]
    return max(tops) if tops else 0.0


def _place_enum_nodes(
    tree_nodes: list[_AnyNode],
    enum_registry: dict[str, _EnumNode],
) -> None:
    """
    Place all enum boxes in a single vertical column to the right of the tree.

    Enums are sorted so that those first referenced at a higher tree level
    appear nearer the top of the column.  All boxes share the same x coordinate
    so they never overlap the tree or each other.

    :param tree_nodes: Flat list of all placed tree nodes.
    :param enum_registry: Enum boxes keyed by type name (x/y will be set here).
    """
    for enum_node in enum_registry.values():
        enum_node.height = _enum_node_height(enum_node)

    if not enum_registry:
        return

    column_x = max(n.x + n.width for n in tree_nodes) + _ENUM_GAP

    sorted_enums = sorted(
        enum_registry.values(),
        key=lambda e: _first_reference_depth(e.type_name, tree_nodes),
        reverse=True,
    )

    top_anchor = max(n.y + n.height for n in tree_nodes)
    cursor_y = top_anchor
    gap = _COL_GAP * 0.5
    for enum_node in sorted_enums:
        enum_node.x = column_x
        enum_node.y = cursor_y - enum_node.height
        cursor_y -= enum_node.height + gap


# ── drawing primitives ────────────────────────────────────────────────────────


def _filled_rect(ax: Axes, x: float, y: float, w: float, h: float,
                 fc: str, ec: str = "none", lw: float = 0.0, z: int = 2) -> None:
    ax.add_patch(Rectangle((x, y), w, h, facecolor=fc, edgecolor=ec,
                            linewidth=lw, zorder=z))


def _border(ax: Axes, x: float, y: float, w: float, h: float,
            ec: str, lw: float = _BW, z: int = 6) -> None:
    ax.add_patch(Rectangle((x, y), w, h, facecolor="none", edgecolor=ec,
                            linewidth=lw, zorder=z))


def _hline(ax: Axes, x: float, y: float, w: float, color: str,
           lw: float = 1.0, z: int = 3) -> None:
    ax.plot([x, x + w], [y, y], color=color, linewidth=lw, zorder=z)


# ── shared row drawing ────────────────────────────────────────────────────────


def _draw_attr_row(
    ax: Axes,
    x: float,
    row_y: float,
    w: float,
    attr: _Attribute,
    row_bg: str,
) -> float:
    """
    Draw one attribute row and return the y coordinate after the row.

    The attribute name is left-aligned and the type tag is right-aligned
    inside the box, eliminating any dependency on text-width estimates.

    :param ax: Matplotlib axes.
    :param x: Left edge of the containing box.
    :param row_y: Vertical centre of the name/type line.
    :param w: Width of the containing box.
    :param attr: The attribute to render.
    :param row_bg: Background fill colour for this row.
    :return: New row_y after consuming this row's vertical space.
    """
    _filled_rect(ax, x, row_y - _LH / 2, w, _LH, row_bg)
    type_color = _TYPE_COLOR.get(attr.type_name, "#555")
    type_label = attr.enum_type_name or _TYPE_SHORT.get(attr.type_name, attr.type_name)
    ax.text(x + 0.20, row_y, attr.name,
            fontsize=6.5, color="#17202A", va="center", ha="left",
            fontfamily="sans-serif", zorder=4)
    ax.text(x + w - 0.15, row_y, f"{type_label}",
            fontsize=6.5, color=type_color, va="center", ha="right",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)
    return row_y - _LH


def _draw_rows(
    ax: Axes,
    x: float,
    top: float,
    w: float,
    attrs: list[_Attribute],
    style: _NodeStyle,
) -> None:
    """Draw the alternating-row attribute table, starting just below *top*."""
    _hline(ax, x, top, w, style.border, lw=1.0)
    row_y = top - _ROW_PAD_V - _LH / 2
    for i, attr in enumerate(attrs):
        row_bg = style.row_odd if i % 2 == 0 else style.row_even
        row_y = _draw_attr_row(ax, x, row_y, w, attr, row_bg)


# ── node drawing ──────────────────────────────────────────────────────────────


def _draw_header(
    ax: Axes,
    x: float,
    top: float,
    w: float,
    title: str,
    stereotype: str,
    style: _NodeStyle,
    title_fontsize: float = 9.0,
) -> None:
    """Draw the coloured header band with a title and UML stereotype line."""
    _filled_rect(ax, x, top - _HEADER_H, w, _HEADER_H, style.header, z=3)
    ax.text(x + w / 2, top - _HEADER_H * 0.28, title,
            fontsize=title_fontsize, color=style.header_fg, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)
    if stereotype:
        ax.text(x + w / 2, top - _HEADER_H * 0.78, stereotype,
                fontsize=6, color=style.stereo_fg, ha="center", va="center",
                fontfamily="sans-serif", fontstyle="italic", zorder=4)


def _draw_class_node(ax: Axes, node: _ClassNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    _draw_header(ax, x, top, w, node.class_name, "<<circuit>>", _STYLE_CLASS,
                 title_fontsize=10.0)

    cursor = top - _HEADER_H
    if node.attributes:
        _draw_rows(ax, x, cursor, w, node.attributes, _STYLE_CLASS)
        cursor -= _ROW_PAD_V + _rows_height(len(node.attributes)) + _ROW_PAD_V

    # Three-cell stat band: sum nodes (green) | product nodes (blue) | leaf nodes (purple)
    _hline(ax, x, cursor, w, _STYLE_CLASS.border, lw=0.8)
    bar_y = cursor - _STAT_BAR_H
    third = w / 3
    stats = [
        (_CC_STAT_SUM,  "sum",     node.stats.num_sum),
        (_CC_STAT_PROD, "product", node.stats.num_product),
        (_CC_STAT_LEAF, "leaf",    node.stats.num_leaf),
    ]
    for i, (color, label, value) in enumerate(stats):
        cell_x = x + i * third
        _filled_rect(ax, cell_x, bar_y, third, _STAT_BAR_H, color, z=3)
        if i > 0:
            ax.plot([cell_x, cell_x], [bar_y, cursor],
                    color="white", linewidth=0.8, zorder=4)
        mid_x = cell_x + third / 2
        ax.text(mid_x, bar_y + _STAT_BAR_H * 0.66, str(value),
                fontsize=9, color=_CC_STAT_FG, ha="center", va="center",
                fontfamily="sans-serif", fontweight="bold", zorder=4)
        ax.text(mid_x, bar_y + _STAT_BAR_H * 0.18, label,
                fontsize=5, color=_CC_STAT_FG, ha="center", va="center",
                fontfamily="sans-serif", alpha=0.88, zorder=4)

    _border(ax, x, y, w, h, _STYLE_CLASS.border)


def _draw_attr_node(ax: Axes, node: _AttrNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h
    _draw_header(ax, x, top, w, node.label, "", _STYLE_ATTR, title_fontsize=8.0)
    _draw_rows(ax, x, top - _HEADER_H, w, node.attributes, _STYLE_ATTR)
    _border(ax, x, y, w, h, _STYLE_ATTR.border, lw=_THIN_BW)


def _draw_edt_node(ax: Axes, node: _EDTNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h
    _draw_header(ax, x, top, w, node.relation_name, "<<template>>", _STYLE_EDT)
    _draw_rows(ax, x, top - _HEADER_H, w, node.latent_attributes, _STYLE_EDT)
    _border(ax, x, y, w, h, _STYLE_EDT.border, lw=_THIN_BW)


def _draw_enum_node(ax: Axes, node: _EnumNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h
    _draw_header(ax, x, top, w, node.type_name, "<<enumeration>>", _STYLE_ENUM,
                 title_fontsize=8.0)
    _hline(ax, x, top - _HEADER_H, w, _STYLE_ENUM.border, lw=1.0)
    row_y = top - _HEADER_H - _ROW_PAD_V - _LH / 2
    for i, value in enumerate(node.values):
        row_bg = _STYLE_ENUM.row_odd if i % 2 == 0 else _STYLE_ENUM.row_even
        _filled_rect(ax, x, row_y - _LH / 2, w, _LH, row_bg)
        ax.text(x + 0.22, row_y, value,
                fontsize=6.5, color=_STYLE_ENUM.header, va="center", ha="left",
                fontfamily="monospace", zorder=4)
        row_y -= _LH
    _border(ax, x, y, w, h, _STYLE_ENUM.border, lw=_THIN_BW)


# ── arrow drawing ─────────────────────────────────────────────────────────────


def _diamond(ax: Axes, cx: float, cy: float, color: str, size: float = 0.15) -> None:
    pts = np.array([
        [cx,               cy + size],
        [cx + size * 0.55, cy],
        [cx,               cy - size],
        [cx - size * 0.55, cy],
    ])
    ax.fill(pts[:, 0], pts[:, 1], color=color, zorder=5)


def _open_arrow_up(ax: Axes, tip_x: float, tip_y: float, color: str) -> None:
    ax.annotate("", xy=(tip_x, tip_y + 0.01), xytext=(tip_x, tip_y + 0.30),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2,
                                mutation_scale=9),
                zorder=4)


def _elbow(ax: Axes, x1: float, y1: float, x2: float, y2: float,
           color: str, lw: float = 1.2) -> None:
    if abs(x1 - x2) < 0.05:
        ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw,
                zorder=2, solid_capstyle="round")
    else:
        mid = (y1 + y2) / 2
        ax.plot([x1, x1, x2, x2], [y1, mid, mid, y2],
                color=color, linewidth=lw, zorder=2, solid_capstyle="round")


def _dashed_use_arrow(ax: Axes, x1: float, y1: float,
                      x2: float, y2: float, color: str = _CARR_ENUM) -> None:
    """Draw a dashed ``<<use>>`` dependency arrow from (x1, y1) to (x2, y2)."""
    ax.annotate(
        "", xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.0,
                        linestyle="dashed", mutation_scale=8),
        zorder=2,
    )


def _mult_label(ax: Axes, x: float, y: float, text: str, color: str = _CMULT) -> None:
    ax.text(x, y, text, fontsize=8, color=color, va="center",
            fontweight="bold", fontfamily="sans-serif", zorder=4)


def _draw_arrows(ax: Axes, node: _AnyNode) -> None:
    """Recursively draw all structural arrows in the tree.

    All arrows exit from the bottom-centre of the parent and enter the
    top-centre of the child, so lines never cross node bodies.
    """
    match node:
        case _AttrNode():
            pass

        case _EDTNode():
            # EDT bottom → child top
            ex, ey = node.x + node.width / 2, node.y
            cx, cy = node.child.x + node.child.width / 2, node.child.y + node.child.height
            _elbow(ax, ex, ey, cx, cy, _CARR_EDT)
            _open_arrow_up(ax, cx, cy, _CARR_EDT)
            _mult_label(ax, (ex + cx) / 2 + 0.12, (ey + cy) / 2, "*")
            _draw_arrows(ax, node.child)

        case _ClassNode():
            # class bottom → sub-object top
            for sub in node.sub_object_nodes:
                px, py = node.x + node.width / 2, node.y
                sx, sy = sub.x + sub.width / 2, sub.y + sub.height
                _elbow(ax, px, py, sx, sy, _CARR_COMP)
                _diamond(ax, px, py, _CARR_COMP)
            # class bottom → EDT top
            for edt in node.edt_children:
                # If there are sub-object nodes, arrows originate from their bottoms
                if node.sub_object_nodes:
                    # find the sub-object node that is horizontally closest to this EDT
                    closest_sub = min(
                        node.sub_object_nodes,
                        key=lambda s: abs(s.x + s.width / 2 - (edt.x + edt.width / 2)),
                    )
                    px = closest_sub.x + closest_sub.width / 2
                    py = closest_sub.y
                else:
                    px = node.x + node.width / 2
                    py = node.y
                ex, ey = edt.x + edt.width / 2, edt.y + edt.height
                _elbow(ax, px, py, ex, ey, _CARR_EDT, lw=1.4)
                _mult_label(ax, (px + ex) / 2 + 0.12, (py + ey) / 2, "1")
                _draw_arrows(ax, edt)


def _draw_nodes(ax: Axes, node: _AnyNode) -> None:
    """Recursively draw all node boxes."""
    match node:
        case _AttrNode():
            _draw_attr_node(ax, node)
        case _EDTNode():
            _draw_edt_node(ax, node)
            _draw_nodes(ax, node.child)
        case _ClassNode():
            _draw_class_node(ax, node)
            for a in node.sub_object_nodes:
                _draw_nodes(ax, a)
            for e in node.edt_children:
                _draw_nodes(ax, e)


def _enum_refs(node: _AnyNode) -> set[str]:
    """Return enum type names referenced by attributes in *node*."""
    match node:
        case _ClassNode():
            attrs = node.attributes
        case _AttrNode():
            attrs = node.attributes
        case _EDTNode():
            attrs = node.latent_attributes
        case _:
            return set()
    return {a.enum_type_name for a in attrs if a.enum_type_name}


def _draw_enum_use_arrows(
    ax: Axes,
    tree_nodes: list[_AnyNode],
    enum_registry: dict[str, _EnumNode],
) -> None:
    """
    Draw dashed ``<<use>>`` arrows from every node that references an enum
    to the corresponding :class:`_EnumNode` box.

    :param tree_nodes: Pre-collected flat list of all tree nodes (from :func:`_collect`).
    :param enum_registry: Enum boxes keyed by type name.
    """
    for node in tree_nodes:
        for enum_name in _enum_refs(node):
            if enum_name not in enum_registry:
                continue
            enum_node = enum_registry[enum_name]
            src_x = node.x + node.width
            src_y = node.y + node.height / 2
            dst_x = enum_node.x
            dst_y = enum_node.y + enum_node.height / 2
            _dashed_use_arrow(ax, src_x, src_y, dst_x, dst_y)


# ── legend ────────────────────────────────────────────────────────────────────

_LEGEND_STYLES: list[_NodeStyle] = [_STYLE_CLASS, _STYLE_ATTR, _STYLE_EDT, _STYLE_ENUM]


def _draw_legend(ax: Axes, x: float, y: float) -> None:
    for i, style in enumerate(_LEGEND_STYLES):
        if not style.legend_label:
            continue
        rx = x + i * 5.2
        ax.add_patch(Rectangle((rx, y - 0.12), 0.28, 0.28,
                                facecolor=style.header, edgecolor="none", zorder=3))
        ax.text(rx + 0.38, y + 0.02, style.legend_label,
                fontsize=7, color="#5D6D7E", va="center", fontfamily="sans-serif")

    ty = y - 0.55
    ax.text(x, ty + 0.24, "Attribute types:",
            fontsize=7, color="#5D6D7E", fontweight="bold",
            va="center", fontfamily="sans-serif")
    off = 0.0
    for type_name, short in _TYPE_SHORT.items():
        color = _TYPE_COLOR[type_name]
        ax.text(x + off, ty, short,
                fontsize=7, color=color, va="center",
                fontfamily="sans-serif", fontstyle="italic",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#F0F0F0",
                          edgecolor=color, linewidth=0.8))
        ax.text(x + off + 0.32, ty, f"= {type_name}  ",
                fontsize=7, color="#5D6D7E", va="center", fontfamily="sans-serif")
        off += 2.2


# ── public API ────────────────────────────────────────────────────────────────


@dataclass
class RSPNPosterPlotter:
    """
    Renders a fitted :class:`~probabilistic_model.probabilistic_circuit.relational.rspn.RelationalProbabilisticCircuit`
    as a UML-style class diagram in SVG (or PNG / PDF).

    Visual language is inspired by the giskard motion-statechart plotter:
    thick borders, coloured info bands, alternating-row attribute tables.

    .. note::
        The RSPN must be fitted before calling :meth:`save` or :meth:`show`.

    Example usage::

        plotter = RSPNPosterPlotter(rspn)
        plotter.save("rspn_diagram.svg")
    """

    rspn: RelationalProbabilisticCircuit
    """The fitted relational probabilistic circuit to visualise."""

    padding: float = 1.0
    """White-space margin around the diagram in figure units."""

    dpi: int = 150
    """Resolution used for raster output (ignored for SVG)."""

    def _build(self) -> Figure:
        enum_registry: dict[str, _EnumNode] = {}
        root = _build_tree(self.rspn, set(), enum_registry)
        _assign_heights(root)
        _place(root, _subtree_w(root) / 2, 0.0)

        tree_nodes = _collect(root)
        _place_enum_nodes(tree_nodes, enum_registry)
        enum_nodes = list(enum_registry.values())

        all_nodes: list = tree_nodes + enum_nodes
        bx0, by0, bx1, by1 = _bounds(all_nodes)

        pad = self.padding
        fig_w = max((bx1 - bx0) + 2 * pad, 8.0)
        fig_h = (by1 - by0) + 2 * pad + 1.3

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=self.dpi)
        ax.set_xlim(bx0 - pad, bx1 + pad)
        ax.set_ylim(by0 - pad - 1.2, by1 + pad)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.patch.set_facecolor("white")

        ax.text(
            (bx0 + bx1) / 2, by1 + pad * 0.65,
            f"Relational Probabilistic Circuit  —  {self.rspn.class_.__name__}",
            fontsize=11, fontweight="bold", color=_STYLE_CLASS.header,
            ha="center", va="center", fontfamily="sans-serif",
        )

        # Draw arrows first (behind nodes), then nodes on top
        _draw_arrows(ax, root)
        _draw_enum_use_arrows(ax, tree_nodes, enum_registry)
        _draw_nodes(ax, root)
        for enum_node in enum_nodes:
            _draw_enum_node(ax, enum_node)

        # Legend anchored to the figure bottom-left in axes coordinates
        legend_y = by0 - pad * 0.35
        _draw_legend(ax, bx0, legend_y)

        fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.01)
        return fig

    def save(self, path: str) -> None:
        """
        Render and save the diagram.

        The output format is inferred from the file extension
        (``.svg``, ``.png``, ``.pdf``).

        :param path: Destination file path.
        """
        fig = self._build()
        fig.savefig(path, format=path.rsplit(".", 1)[-1],
                    bbox_inches="tight", facecolor=fig.get_facecolor())
        plt.close(fig)

    def show(self) -> None:
        """Display the diagram interactively via :func:`matplotlib.pyplot.show`."""
        matplotlib.use("TkAgg")
        fig = self._build()
        plt.show()
        plt.close(fig)

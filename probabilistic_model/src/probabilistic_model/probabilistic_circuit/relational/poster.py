"""
UML-style class diagram for :class:`RelationalProbabilisticCircuit`.

Visual language is inspired by the giskard motion-statechart plotter
(thick borders, coloured info bands, structured HTML-table compartments).

Node kinds:

* **Class node** (``<<circuit>>``) — header + alternating-row attribute
  table + three-cell stat band (parameters | variables | nodes).
* **Sub-object node** — one per nested value-object group (e.g.
  ``orientation``), connected to its class by a composition arrow.
* **Template node** (``<<template>>``) — one per exchangeable-distribution
  template; sits between parent and child class, listing the latent
  conditioning variables.

Output: a single SVG file (or PNG / PDF).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Union

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
import numpy as np

matplotlib.use("Agg")

from random_events.variable import Continuous, Symbolic, Integer

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
    )


# ── palette (giskard-inspired) ────────────────────────────────────────────────

# Class node
_CC_HEADER: str = "#1B2631"       # dark navy header
_CC_HEADER_FG: str = "#FFFFFF"
_CC_STEREO: str = "#85C1E9"       # light-blue stereotype
_CC_STAT_A: str = "#1A9641"       # green  — parameters
_CC_STAT_B: str = "#2B83BA"       # blue   — variables
_CC_STAT_C: str = "#6C3483"       # purple — nodes
_CC_STAT_FG: str = "#FFFFFF"
_CC_ROW_ODD: str = "#EBF5FB"
_CC_ROW_EVEN: str = "#D6EAF8"
_CC_BORDER: str = "#1B2631"

# Attribute node
_CA_HEADER: str = "#1A5276"
_CA_HEADER_FG: str = "#FFFFFF"
_CA_ROW_ODD: str = "#EBF5FB"
_CA_ROW_EVEN: str = "#D6EAF8"
_CA_BORDER: str = "#1A5276"

# Template (EDT) node
_CT_HEADER: str = "#6E2F1A"
_CT_HEADER_FG: str = "#FFFFFF"
_CT_STEREO: str = "#F0B27A"
_CT_ROW_ODD: str = "#FDF2E9"
_CT_ROW_EVEN: str = "#FAE5D3"
_CT_BORDER: str = "#6E2F1A"

# Arrows / multiplicity
_CARR_COMP: str = "#1B2631"   # composition (class → attribute node)
_CARR_EDT: str = "#6E2F1A"    # class → template  and  template → child class
_CMULT: str = "#C0392B"

_ENUM_LH: float = 0.22   # extra height per enum attribute (for the values line)

_TYPE_SHORT: dict[str, str] = {
    "Continuous": "float",
    "Integer": "int",
    "Symbolic": "<<enum>>",
}
_TYPE_COLOR: dict[str, str] = {
    "Continuous": "#1A9641",
    "Integer": "#2B83BA",
    "Symbolic": "#7B2D8B",
}

# ── geometry ──────────────────────────────────────────────────────────────────

_CLASS_W: float = 4.4
_ATTR_W: float = 3.6
_EDT_W: float = 4.2

_LH: float = 0.29            # row height
_HEADER_H: float = 0.76
_STAT_BAR_H: float = 0.40    # two-column stat band
_ROW_PAD_V: float = 0.10     # extra vertical padding above/below row block
_BW: float = 2.8             # border (line)width in points — giskard-style thick

_COL_GAP: float = 1.0
_ROW_GAP: float = 2.0


# ── data model ────────────────────────────────────────────────────────────────


@dataclass
class _Attribute:
    name: str
    type_name: str
    enum_values: list[str] = field(default_factory=list)
    """For Symbolic variables, the list of enum member names."""


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
    """
    An exchangeable-distribution template box sitting between parent and child.
    """

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
    alternating-row table, and a three-cell stat band (parameters |
    variables | nodes).  Sub-object groups and EDT children are rendered
    as separate connected nodes.
    """

    class_name: str
    num_parameters: int
    """Total learnable parameters across all leaf distributions and sum weights."""

    num_variables: int
    """Number of variables modelled by the class circuit."""

    num_nodes: int
    """Total node count in the circuit DAG."""

    attributes: list[_Attribute]
    """All scalar attributes shown inside the class box."""

    sub_object_nodes: list[_AttrNode]
    """One association box per sub-object group (e.g. orientation, position)."""

    edt_children: list[_EDTNode]

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_CLASS_W, init=False)
    height: float = field(default=0.0, init=False)


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


def _count_parameters(rspn: RelationalProbabilisticCircuit) -> int:
    """
    Count the total number of learnable parameters in a class circuit.

    Includes leaf distribution parameters and sum-node mixture weights
    (one free weight per child minus one, since weights sum to one).

    :param rspn: Fitted RSPN whose class circuit to inspect.
    :return: Total parameter count.
    """
    pc = rspn.class_probabilistic_circuit
    total = 0
    for node in pc.nodes():
        if isinstance(node, SumUnit):
            k = len(list(pc.graph.successors(node.index)))
            total += max(k - 1, 0)
    for leaf in pc.leaves:
        d = leaf.distribution
        if hasattr(d, "probabilities"):
            total += len(d.probabilities)
        elif hasattr(d, "location"):
            total += 1
        else:
            total += 1
    return total


def _build_tree(
    rspn: RelationalProbabilisticCircuit,
    latent_names: set[str],
) -> _ClassNode:
    """
    Recursively extract the diagram node tree from a fitted RSPN.

    Scalar variables with a sub-object prefix (e.g. ``orientation.w``) are
    separated into :class:`_AttrNode` sub-object boxes; the remaining flat
    attributes and aggregation statistics are listed directly in the class
    box.  Latent conditioning variables shared with the parent circuit are
    excluded.

    :param rspn: The fitted RSPN to visualise.
    :param latent_names: Variable names to suppress in this class.
    :return: Root :class:`_ClassNode` of the diagram tree.
    """
    pc = rspn.class_probabilistic_circuit
    class_name = rspn.class_.__name__

    agg_names: set[str] = set()
    if rspn.feature_extractor is not None:
        for variables in rspn.feature_extractor.exchangeable_features.values():
            agg_names.update(v._name_ for v in variables)

    sub_groups: dict[str, list[_Attribute]] = {}
    class_box_attrs: list[_Attribute] = []

    for var in pc.variables:
        if var.name in latent_names:
            continue
        display = _strip_prefix(var.name, class_name)
        enum_values: list[str] = []
        if isinstance(var, Symbolic):
            enum_values = [str(v).split(".")[-1] for v in list(var.domain)]
        attr = _Attribute(name=display, type_name=_var_type(var), enum_values=enum_values)
        if var.name in agg_names:
            continue
        parts = display.split(".")
        if len(parts) >= 2:
            sub_groups.setdefault(parts[0], []).append(
                _Attribute(name=".".join(parts[1:]), type_name=attr.type_name)
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
        child_node = _build_tree(template.template_distribution, child_latent_names)
        edt_children.append(
            _EDTNode(
                relation_name=rel_name,
                latent_attributes=latent_attrs,
                child=child_node,
            )
        )

    return _ClassNode(
        class_name=class_name,
        num_parameters=_count_parameters(rspn),
        num_variables=len(pc.variables),
        num_nodes=len(pc.nodes()),
        attributes=class_box_attrs,
        sub_object_nodes=sub_object_nodes,
        edt_children=edt_children,
    )


# ── height calculation ────────────────────────────────────────────────────────


def _attrs_height(attrs: list[_Attribute]) -> float:
    return sum(_LH + (_ENUM_LH if a.enum_values else 0.0) for a in attrs)


def _attr_node_height(n: _AttrNode) -> float:
    return _HEADER_H + _ROW_PAD_V + _attrs_height(n.attributes) + _ROW_PAD_V


def _edt_node_height(n: _EDTNode) -> float:
    inner = _attrs_height(n.latent_attributes) if n.latent_attributes else _LH
    return _HEADER_H + _ROW_PAD_V + inner + _ROW_PAD_V


def _class_node_height(n: _ClassNode) -> float:
    inner = _attrs_height(n.attributes)
    return (
        _HEADER_H
        + (_ROW_PAD_V + inner + _ROW_PAD_V if n.attributes else 0.0)
        + _STAT_BAR_H
    )


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


# ── layout ────────────────────────────────────────────────────────────────────


def _subtree_w(node: _AnyNode) -> float:
    match node:
        case _AttrNode():
            return node.width
        case _EDTNode():
            return max(node.width, _subtree_w(node.child))
        case _ClassNode():
            all_ch: list[_AnyNode] = list(node.sub_object_nodes) + list(node.edt_children)
            if not all_ch:
                return node.width
            ch_total = sum(_subtree_w(c) for c in all_ch) + _COL_GAP * (len(all_ch) - 1)
            return max(node.width, ch_total)


def _place(node: _AnyNode, x_ctr: float, y_top: float) -> None:
    """Recursively assign positions; y_top is the top edge of *node*."""
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
            all_ch: list[_AnyNode] = list(node.sub_object_nodes) + list(node.edt_children)
            if not all_ch:
                return
            total_w = sum(_subtree_w(c) for c in all_ch) + _COL_GAP * (len(all_ch) - 1)
            cursor = x_ctr - total_w / 2
            child_top = node.y - _ROW_GAP
            for ch in all_ch:
                sw = _subtree_w(ch)
                _place(ch, cursor + sw / 2, child_top)
                cursor += sw + _COL_GAP


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
    return result


def _bounds(nodes: list[_AnyNode]) -> tuple[float, float, float, float]:
    return (
        min(n.x for n in nodes),
        min(n.y for n in nodes),
        max(n.x + n.width for n in nodes),
        max(n.y + n.height for n in nodes),
    )


# ── drawing primitives ────────────────────────────────────────────────────────


def _filled_rect(ax, x, y, w, h, fc, ec="none", lw=0.0, z=2):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=fc, edgecolor=ec,
                            linewidth=lw, zorder=z))


def _border(ax, x, y, w, h, ec, lw=_BW, z=6):
    ax.add_patch(Rectangle((x, y), w, h, facecolor="none", edgecolor=ec,
                            linewidth=lw, zorder=z))


def _hline(ax, x, y, w, color, lw=1.0, z=3):
    ax.plot([x, x + w], [y, y], color=color, linewidth=lw, zorder=z)


# ── shared row drawing ───────────────────────────────────────────────────────


def _draw_attr_row(
    ax,
    x: float,
    row_y: float,
    w: float,
    attr: _Attribute,
    row_bg: str,
    index: int,
) -> float:
    """
    Draw one attribute row and return the y coordinate after the row.

    For Symbolic variables an extra line listing the enum values
    (``{VALUE_A, VALUE_B}``) is rendered below the name/type line,
    following standard UML enumeration notation.

    :param ax: Matplotlib axes.
    :param x: Left edge of the containing box.
    :param row_y: Vertical centre of the first (name/type) line.
    :param w: Width of the containing box.
    :param attr: The attribute to render.
    :param row_bg: Background fill colour for this row.
    :param index: Row index (unused here, kept for parity with callers).
    :return: New row_y after consuming this attribute's vertical space.
    """
    row_h = _LH + (_ENUM_LH if attr.enum_values else 0.0)
    _filled_rect(ax, x, row_y - _LH / 2, w, row_h, row_bg, z=2)

    type_short = _TYPE_SHORT.get(attr.type_name, attr.type_name)
    type_color = _TYPE_COLOR.get(attr.type_name, "#555")
    name_w = len(attr.name) * 0.057
    ax.text(x + 0.20, row_y, attr.name,
            fontsize=6.5, color="#17202A", va="center", ha="left",
            fontfamily="sans-serif", zorder=4)
    ax.text(x + 0.20 + name_w, row_y, f" : {type_short}",
            fontsize=6.5, color=type_color, va="center", ha="left",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)

    if attr.enum_values:
        values_str = "{" + ", ".join(attr.enum_values) + "}"
        ax.text(x + 0.36, row_y - _LH * 0.72, values_str,
                fontsize=5.5, color=type_color, va="center", ha="left",
                fontfamily="monospace", fontstyle="italic", zorder=4)

    return row_y - row_h


# ── class node drawing ────────────────────────────────────────────────────────


def _draw_class_node(ax, node: _ClassNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    # ── header band ───────────────────────────────────────────────────────
    _filled_rect(ax, x, top - _HEADER_H, w, _HEADER_H, _CC_HEADER, z=3)
    ax.text(x + w / 2, top - _HEADER_H * 0.28, node.class_name,
            fontsize=10, color=_CC_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)
    ax.text(x + w / 2, top - _HEADER_H * 0.78, "<<circuit>>",
            fontsize=6, color=_CC_STEREO, ha="center", va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)

    # ── attribute rows ────────────────────────────────────────────────────
    cursor = top - _HEADER_H
    if node.attributes:
        _hline(ax, x, cursor, w, _CC_BORDER, lw=0.8)
        row_y = cursor - _ROW_PAD_V - _LH / 2
        for i, attr in enumerate(node.attributes):
            row_bg = _CC_ROW_ODD if i % 2 == 0 else _CC_ROW_EVEN
            row_y = _draw_attr_row(ax, x, row_y, w, attr, row_bg, i)
        cursor -= _ROW_PAD_V + _attrs_height(node.attributes) + _ROW_PAD_V

    # ── three-cell stat band ──────────────────────────────────────────────
    # parameters (green) | variables (blue) | nodes (purple)
    _hline(ax, x, cursor, w, _CC_BORDER, lw=0.8)
    bar_y = cursor - _STAT_BAR_H
    third = w / 3
    stats = [
        (_CC_STAT_A, "params",    node.num_parameters),
        (_CC_STAT_B, "variables", node.num_variables),
        (_CC_STAT_C, "nodes",     node.num_nodes),
    ]
    for i, (color, label, value) in enumerate(stats):
        cell_x = x + i * third
        _filled_rect(ax, cell_x, bar_y, third, _STAT_BAR_H, color, z=3)
        if i > 0:
            ax.plot([cell_x, cell_x], [bar_y, cursor],
                    color="white", linewidth=0.8, zorder=4)
        mid_x = cell_x + third / 2
        mid_y = bar_y + _STAT_BAR_H / 2
        ax.text(mid_x, mid_y + _STAT_BAR_H * 0.16, str(value),
                fontsize=9, color=_CC_STAT_FG, ha="center", va="center",
                fontfamily="sans-serif", fontweight="bold", zorder=4)
        ax.text(mid_x, bar_y + _STAT_BAR_H * 0.82, label,
                fontsize=5, color=_CC_STAT_FG, ha="center", va="center",
                fontfamily="sans-serif", alpha=0.88, zorder=4)

    _border(ax, x, y, w, h, _CC_BORDER)


# ── attribute node drawing ────────────────────────────────────────────────────


def _draw_attr_node(ax, node: _AttrNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    # header
    _filled_rect(ax, x, top - _HEADER_H, w, _HEADER_H, _CA_HEADER, z=3)
    ax.text(x + w / 2, top - _HEADER_H / 2, node.label,
            fontsize=8, color=_CA_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)

    # alternating-row attribute table
    _hline(ax, x, top - _HEADER_H, w, _CA_BORDER, lw=1.0)
    row_y = top - _HEADER_H - _ROW_PAD_V - _LH / 2
    for i, attr in enumerate(node.attributes):
        row_bg = _CA_ROW_ODD if i % 2 == 0 else _CA_ROW_EVEN
        row_y = _draw_attr_row(ax, x, row_y, w, attr, row_bg, i)

    _border(ax, x, y, w, h, _CA_BORDER)


# ── EDT node drawing ──────────────────────────────────────────────────────────


def _draw_edt_node(ax, node: _EDTNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    # header
    _filled_rect(ax, x, top - _HEADER_H, w, _HEADER_H, _CT_HEADER, z=3)
    ax.text(x + w / 2, top - _HEADER_H * 0.28, node.relation_name,
            fontsize=9, color=_CT_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)
    ax.text(x + w / 2, top - _HEADER_H * 0.78, "<<template>>",
            fontsize=6, color=_CT_STEREO, ha="center", va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)

    _hline(ax, x, top - _HEADER_H, w, _CT_BORDER, lw=1.0)
    row_y = top - _HEADER_H - _ROW_PAD_V - _LH / 2
    for i, attr in enumerate(node.latent_attributes):
        row_bg = _CT_ROW_ODD if i % 2 == 0 else _CT_ROW_EVEN
        row_y = _draw_attr_row(ax, x, row_y, w, attr, row_bg, i)

    _border(ax, x, y, w, h, _CT_BORDER)


# ── arrow drawing ─────────────────────────────────────────────────────────────


def _diamond(ax, cx, cy, color, size=0.15):
    pts = np.array([
        [cx,                cy + size],
        [cx + size * 0.55,  cy],
        [cx,                cy - size],
        [cx - size * 0.55,  cy],
    ])
    ax.fill(pts[:, 0], pts[:, 1], color=color, zorder=5)


def _open_arrow_up(ax, tip_x, tip_y, color):
    ax.annotate("", xy=(tip_x, tip_y + 0.01), xytext=(tip_x, tip_y + 0.30),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.2,
                                mutation_scale=9),
                zorder=4)


def _elbow(ax, x1, y1, x2, y2, color, lw=1.2):
    if abs(x1 - x2) < 0.05:
        ax.plot([x1, x2], [y1, y2], color=color, linewidth=lw,
                zorder=2, solid_capstyle="round")
    else:
        mid = (y1 + y2) / 2
        ax.plot([x1, x1, x2, x2], [y1, mid, mid, y2],
                color=color, linewidth=lw, zorder=2, solid_capstyle="round")


def _mult_label(ax, x, y, text, color=_CMULT):
    ax.text(x, y, text, fontsize=8, color=color, va="center",
            fontweight="bold", fontfamily="sans-serif", zorder=4)


def _draw_arrows(ax, node: _AnyNode) -> None:
    """Recursively draw all arrows in the tree."""
    match node:
        case _AttrNode():
            pass

        case _EDTNode():
            ex, ey = node.x + node.width / 2, node.y
            cx, cy = node.child.x + node.child.width / 2, node.child.y + node.child.height
            _elbow(ax, ex, ey, cx, cy, _CARR_EDT)
            _open_arrow_up(ax, cx, cy, _CARR_EDT)
            _mult_label(ax, (ex + cx) / 2 + 0.12, (ey + cy) / 2, "*")
            _draw_arrows(ax, node.child)

        case _ClassNode():
            px, py = node.x + node.width / 2, node.y
            for sub in node.sub_object_nodes:
                sx = sub.x + sub.width / 2
                sy = sub.y + sub.height
                _elbow(ax, px, py, sx, sy, _CARR_COMP)
                _diamond(ax, px, py, _CARR_COMP)

            for edt in node.edt_children:
                ex = edt.x + edt.width / 2
                ey = edt.y + edt.height
                _elbow(ax, px, py, ex, ey, _CARR_EDT, lw=1.4)
                _mult_label(ax, (px + ex) / 2 + 0.12, (py + ey) / 2, "1")
                _draw_arrows(ax, edt)


def _draw_nodes(ax, node: _AnyNode) -> None:
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


# ── legend ────────────────────────────────────────────────────────────────────


def _draw_legend(ax, x: float, y: float) -> None:
    kinds = [
        (_CC_HEADER,  "<<circuit>>  class  (name + stats)"),
        (_CA_HEADER,  "attribute group / sub-object"),
        (_CT_HEADER,  "<<template>>  EDT  (latent conditioning vars)"),
    ]
    for i, (color, label) in enumerate(kinds):
        rx = x + i * 5.2
        ax.add_patch(Rectangle((rx, y - 0.12), 0.28, 0.28,
                                facecolor=color, edgecolor="none", zorder=3))
        ax.text(rx + 0.38, y + 0.02, label,
                fontsize=7, color="#5D6D7E", va="center",
                fontfamily="sans-serif")

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
                fontsize=7, color="#5D6D7E", va="center",
                fontfamily="sans-serif")
        off += 2.2


# ── public API ────────────────────────────────────────────────────────────────


@dataclass
class RSPNPosterPlotter:
    """
    Renders a fitted :class:`~probabilistic_model.probabilistic_circuit.relational.rspn.RelationalProbabilisticCircuit`
    as a UML-style class diagram in SVG (or PNG / PDF).

    Visual language is inspired by the giskard motion-statechart plotter:
    thick borders, coloured info bands, alternating-row attribute tables.

    Three node kinds are drawn:

    * **Class nodes** (dark header, ``<<circuit>>``) — minimal: just the class
      name and a two-cell stat band showing cluster count and leaf count.
      No attributes live here; they are in separate association boxes.
    * **Attribute nodes** (blue header) — one per sub-object group (e.g.
      ``orientation``, ``position``) and one catch-all for flat scalars.
      Connected to the owning class by a composition arrow.
    * **Template nodes** (brown header, ``<<template>>``) — one per
      exchangeable-distribution template, listing the latent conditioning
      variables.  Sits between the parent class and the child class.

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
        root = _build_tree(self.rspn, set())
        _assign_heights(root)
        _place(root, _subtree_w(root) / 2, 0.0)

        all_nodes = _collect(root)
        x0, y0, x1, y1 = _bounds(all_nodes)

        pad = self.padding
        fig_w = max((x1 - x0) + 2 * pad, 8.0)
        fig_h = (y1 - y0) + 2 * pad + 1.3

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=self.dpi)
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y0 - pad - 1.2, y1 + pad)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.patch.set_facecolor("white")

        ax.text(
            (x0 + x1) / 2, y1 + pad * 0.65,
            f"Relational Probabilistic Circuit  —  {self.rspn.class_.__name__}",
            fontsize=11, fontweight="bold", color=_CC_HEADER,
            ha="center", va="center", fontfamily="sans-serif",
        )

        # draw arrows first (behind nodes), then nodes on top
        _draw_arrows(ax, root)
        _draw_nodes(ax, root)
        _draw_legend(ax, x0, y0 - pad * 0.55)

        fig.tight_layout(pad=0.1)
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

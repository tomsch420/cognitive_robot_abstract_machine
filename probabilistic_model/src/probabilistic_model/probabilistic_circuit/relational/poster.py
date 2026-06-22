"""
UML-style class diagram explainer for :class:`RelationalProbabilisticCircuit`.

The diagram contains three kinds of nodes:

* **Class boxes** (``<<circuit>>``) — one per RSPN class, showing its direct
  scalar attributes, aggregation statistics, and circuit summary.
* **Sub-object boxes** — one per nested value-object (e.g. ``orientation``,
  ``position``), connected to the owning class with a composition arrow.
* **Template boxes** (``<<template>>``) — one per exchangeable-distribution
  template, sitting between the parent class and the child class, listing the
  latent variables that act as the conditioning bridge.

Output is a single SVG file suitable for printing or documentation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Union

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle, FancyArrowPatch
import numpy as np

matplotlib.use("Agg")

from random_events.variable import Continuous, Symbolic, Integer

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
    )


# ── palette ──────────────────────────────────────────────────────────────────

_C_CLASS_HEADER_BG: str = "#1B2631"
_C_CLASS_HEADER_FG: str = "#FFFFFF"
_C_STEREOTYPE_FG: str = "#85C1E9"

_C_SUBOBJ_HEADER_BG: str = "#1A5276"
_C_SUBOBJ_HEADER_FG: str = "#FFFFFF"

_C_EDT_HEADER_BG: str = "#784212"
_C_EDT_HEADER_FG: str = "#FFFFFF"
_C_EDT_STEREOTYPE_FG: str = "#F0B27A"

_C_SECTION_SCALAR: str = "#EBF5FB"
_C_SECTION_AGG: str = "#FEF9E7"
_C_SECTION_CIRCUIT: str = "#F9F9F9"
_C_SECTION_LATENT: str = "#FDF2E9"

_C_BORDER_CLASS: str = "#1B2631"
_C_BORDER_SUBOBJ: str = "#1A5276"
_C_BORDER_EDT: str = "#784212"

_C_TEXT: str = "#17202A"
_C_MUTED: str = "#5D6D7E"

_C_ARROW_COMP: str = "#1B2631"   # composition (class → sub-object)
_C_ARROW_EDT: str = "#784212"    # class → template
_C_ARROW_INST: str = "#784212"   # template → child class
_C_MULT: str = "#C0392B"

_TYPE_SHORT: dict[str, str] = {
    "Continuous": "float",
    "Integer": "int",
    "Symbolic": "cat",
}
_TYPE_COLOR: dict[str, str] = {
    "Continuous": "#1A9641",
    "Integer": "#2B83BA",
    "Symbolic": "#7B2D8B",
}

# ── geometry ──────────────────────────────────────────────────────────────────

_CLASS_W: float = 4.8
_SUBOBJ_W: float = 3.4
_EDT_W: float = 4.2

_LH: float = 0.30            # line height inside a section
_HEADER_H: float = 0.78
_SEC_PAD_TOP: float = 0.16
_SEC_PAD_BOT: float = 0.10
_COL_GAP: float = 1.0        # horizontal gap between sibling subtrees
_ROW_GAP: float = 2.2        # vertical gap between parent row and child row


# ── attribute data ────────────────────────────────────────────────────────────


@dataclass
class _Attribute:
    """A single modelled variable shown as one row in a compartment."""

    name: str
    type_name: str


# ── node types ────────────────────────────────────────────────────────────────


@dataclass
class _SubObjectNode:
    """
    A value-object whose scalar attributes are grouped under a common prefix.

    For example, variables ``SceneRoom.orientation.{w,x,y,z}`` become one
    ``_SubObjectNode`` named ``orientation``.
    """

    sub_object_name: str
    """Prefix component used to group variables (e.g. ``orientation``)."""

    attributes: list[_Attribute]
    """Scalar attributes belonging to this sub-object."""

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_SUBOBJ_W, init=False)
    height: float = field(default=0.0, init=False)


@dataclass
class _EDTNode:
    """
    An exchangeable-distribution template — the probabilistic bridge between
    a parent class and its child class.

    Displays the latent variables that are jointly modelled in the parent
    circuit and used to condition the child circuit.
    """

    relation_name: str
    """Field name of the relation on the parent class."""

    latent_attributes: list[_Attribute]
    """Aggregation statistics that act as conditioning context."""

    child: _ClassNode
    """The downstream RSPN class this template instantiates."""

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_EDT_W, init=False)
    height: float = field(default=0.0, init=False)


@dataclass
class _ClassNode:
    """
    A fitted RSPN class — the central node type in the diagram.

    Groups its variables into direct scalar attributes (for flat fields),
    aggregation statistics (for relational summaries), and circuit metadata.
    Sub-objects and EDT children are laid out as separate nodes connected by
    edges.
    """

    class_name: str
    num_clusters: int
    num_leaves: int

    direct_attributes: list[_Attribute]
    """Scalar variables that belong directly to the class (not sub-objects)."""

    aggregation_attributes: list[_Attribute]
    """Aggregation statistics jointly modelled with the class attributes."""

    sub_objects: list[_SubObjectNode]
    """Value-object groups extracted from dotted variable names."""

    edt_children: list[_EDTNode]
    """Exchangeable-distribution templates for 1:N relations."""

    x: float = field(default=0.0, init=False)
    y: float = field(default=0.0, init=False)
    width: float = field(default=_CLASS_W, init=False)
    height: float = field(default=0.0, init=False)


_AnyNode = Union[_ClassNode, _SubObjectNode, _EDTNode]


# ── extraction ────────────────────────────────────────────────────────────────


def _var_type(variable) -> str:
    if isinstance(variable, Continuous):
        return "Continuous"
    if isinstance(variable, Symbolic):
        return "Symbolic"
    if isinstance(variable, Integer):
        return "Integer"
    return type(variable).__name__


def _strip_class_prefix(name: str, class_name: str) -> str:
    """Remove leading ``ClassName.`` or ``ClassNameAggregations.`` prefix."""
    for prefix in (f"{class_name}Aggregations.", f"{class_name}."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _build_class_node(
    rspn: RelationalProbabilisticCircuit,
    latent_names: set[str],
) -> _ClassNode:
    """
    Recursively extract a :class:`_ClassNode` tree from an RSPN.

    Variables with three dot-separated components (``Class.sub.attr``) are
    grouped into :class:`_SubObjectNode` instances.  Latent variables shared
    with the parent circuit are excluded from the child's display.

    :param rspn: The fitted RSPN to extract from.
    :param latent_names: Variable names to exclude (latent conditioning vars).
    :return: Populated :class:`_ClassNode` with sub-objects and EDT children.
    """
    pc = rspn.class_probabilistic_circuit
    class_name = rspn.class_.__name__

    num_clusters = sum(1 for n in pc.nodes() if isinstance(n, SumUnit))
    num_leaves = len(pc.leaves)

    agg_names: set[str] = set()
    if rspn.feature_extractor is not None:
        for variables in rspn.feature_extractor.exchangeable_features.values():
            agg_names.update(v._name_ for v in variables)

    # sort variables into: latent (skip), aggregation, direct scalar, sub-object
    sub_object_groups: dict[str, list[_Attribute]] = {}
    direct_attrs: list[_Attribute] = []
    agg_attrs: list[_Attribute] = []

    for var in pc.variables:
        if var.name in latent_names:
            continue

        display = _strip_class_prefix(var.name, class_name)
        attr = _Attribute(name=display, type_name=_var_type(var))

        if var.name in agg_names:
            agg_attrs.append(attr)
        else:
            # detect sub-object grouping: "sub_object.field" (two components remaining)
            parts = display.split(".")
            if len(parts) >= 2:
                group_name = parts[0]
                attr.name = ".".join(parts[1:])
                sub_object_groups.setdefault(group_name, []).append(attr)
            else:
                direct_attrs.append(attr)

    sub_objects = [
        _SubObjectNode(sub_object_name=name, attributes=attrs)
        for name, attrs in sub_object_groups.items()
    ]

    edt_children: list[_EDTNode] = []
    for rel_name, template in rspn.exchangeable_distribution_templates.items():
        child_latent_names = {v.name for v in template.latent_variables}
        latent_attrs = [
            _Attribute(
                name=_strip_class_prefix(v.name, class_name),
                type_name=_var_type(v),
            )
            for v in template.latent_variables
        ]
        child_node = _build_class_node(template.template_distribution, child_latent_names)
        edt_children.append(
            _EDTNode(
                relation_name=rel_name,
                latent_attributes=latent_attrs,
                child=child_node,
            )
        )

    return _ClassNode(
        class_name=class_name,
        num_clusters=num_clusters,
        num_leaves=num_leaves,
        direct_attributes=direct_attrs,
        aggregation_attributes=agg_attrs,
        sub_objects=sub_objects,
        edt_children=edt_children,
    )


# ── height computation ────────────────────────────────────────────────────────


def _section_h(num_rows: int) -> float:
    """Height of one compartment with *num_rows* attribute lines."""
    return _SEC_PAD_TOP + max(num_rows, 1) * _LH + _SEC_PAD_BOT


def _compute_heights(node: _AnyNode) -> None:
    """Recursively compute and assign heights to all nodes in the tree."""
    match node:
        case _SubObjectNode():
            node.height = _HEADER_H + _section_h(len(node.attributes))

        case _EDTNode():
            node.height = _HEADER_H + _section_h(len(node.latent_attributes))
            _compute_heights(node.child)

        case _ClassNode():
            h = (
                _HEADER_H
                + (_section_h(len(node.direct_attributes)) if node.direct_attributes else 0.0)
                + (_section_h(len(node.aggregation_attributes)) if node.aggregation_attributes else 0.0)
                + _section_h(2)   # circuit stats: clusters + leaves
            )
            node.height = h
            for sub in node.sub_objects:
                _compute_heights(sub)
            for edt in node.edt_children:
                _compute_heights(edt)


# ── layout ────────────────────────────────────────────────────────────────────


def _subtree_width(node: _AnyNode) -> float:
    """
    Total horizontal footprint needed to render a node and all its descendants.
    """
    match node:
        case _SubObjectNode():
            return node.width
        case _EDTNode():
            return max(node.width, _subtree_width(node.child))
        case _ClassNode():
            all_children: list[_AnyNode] = list(node.sub_objects) + list(node.edt_children)
            if not all_children:
                return node.width
            children_total = (
                sum(_subtree_width(c) for c in all_children)
                + _COL_GAP * (len(all_children) - 1)
            )
            return max(node.width, children_total)


def _place(node: _AnyNode, x_center: float, y_top: float) -> None:
    """
    Recursively assign ``(x, y)`` positions to every node in the subtree.

    :param node: The root of the subtree to place.
    :param x_center: Horizontal centre of the subtree's allocated column.
    :param y_top: Top edge of this node's allocated row.
    """
    match node:
        case _SubObjectNode():
            node.x = x_center - node.width / 2.0
            node.y = y_top - node.height

        case _EDTNode():
            node.x = x_center - node.width / 2.0
            node.y = y_top - node.height
            _place(node.child, x_center, node.y - _ROW_GAP)

        case _ClassNode():
            node.x = x_center - node.width / 2.0
            node.y = y_top - node.height

            all_children: list[_AnyNode] = list(node.sub_objects) + list(node.edt_children)
            if not all_children:
                return

            total_w = (
                sum(_subtree_width(c) for c in all_children)
                + _COL_GAP * (len(all_children) - 1)
            )
            cursor_x = x_center - total_w / 2.0
            child_y_top = node.y - _ROW_GAP
            for child in all_children:
                sw = _subtree_width(child)
                _place(child, cursor_x + sw / 2.0, child_y_top)
                cursor_x += sw + _COL_GAP


def _collect_all(node: _AnyNode) -> list[_AnyNode]:
    """Gather every node in the tree into a flat list."""
    result: list[_AnyNode] = [node]
    match node:
        case _SubObjectNode():
            pass
        case _EDTNode():
            result.extend(_collect_all(node.child))
        case _ClassNode():
            for sub in node.sub_objects:
                result.extend(_collect_all(sub))
            for edt in node.edt_children:
                result.extend(_collect_all(edt))
    return result


def _bounds(nodes: list[_AnyNode]) -> tuple[float, float, float, float]:
    x0 = min(n.x for n in nodes)
    y0 = min(n.y for n in nodes)
    x1 = max(n.x + n.width for n in nodes)
    y1 = max(n.y + n.height for n in nodes)
    return x0, y0, x1, y1


# ── drawing helpers ───────────────────────────────────────────────────────────


def _rect(ax, x, y, w, h, facecolor, edgecolor="none", lw=0.0, zorder=2):
    ax.add_patch(Rectangle((x, y), w, h, facecolor=facecolor,
                            edgecolor=edgecolor, linewidth=lw, zorder=zorder))


def _hline(ax, x, y, w, color=_C_BORDER_CLASS, lw=0.7):
    ax.plot([x, x + w], [y, y], color=color, linewidth=lw, zorder=3)


def _attr_row(ax, x, y, attr: _Attribute, indent=0.22):
    """Draw ``name : type`` on one row."""
    type_short = _TYPE_SHORT.get(attr.type_name, attr.type_name)
    type_color = _TYPE_COLOR.get(attr.type_name, _C_MUTED)
    name_w = len(attr.name) * 0.057
    ax.text(x + indent, y, attr.name,
            fontsize=6.5, color=_C_TEXT, va="center", ha="left",
            fontfamily="sans-serif", zorder=4)
    ax.text(x + indent + name_w, y, f" : {type_short}",
            fontsize=6.5, color=type_color, va="center", ha="left",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)


def _section(ax, x, cursor_y, w, bg_color, label, attrs: list[_Attribute],
             border_color=_C_BORDER_CLASS) -> float:
    """
    Draw one compartment and return the new cursor_y (bottom of the section).
    """
    h = _section_h(len(attrs))
    sec_y = cursor_y - h
    _rect(ax, x, sec_y, w, h, bg_color)
    _hline(ax, x, cursor_y, w, color=border_color)
    if label:
        ax.text(x + 0.14, cursor_y - _SEC_PAD_TOP * 0.6,
                label, fontsize=5.5, color=_C_MUTED, va="center",
                fontfamily="sans-serif", fontstyle="italic", zorder=4)
    row_y = cursor_y - _SEC_PAD_TOP - _LH / 2
    for attr in attrs:
        _attr_row(ax, x, row_y, attr)
        row_y -= _LH
    return sec_y


def _outer_border(ax, x, y, w, h, color, lw=1.4):
    ax.add_patch(Rectangle((x, y), w, h, facecolor="none",
                            edgecolor=color, linewidth=lw, zorder=5))


# ── node drawing ──────────────────────────────────────────────────────────────


def _draw_class_node(ax, node: _ClassNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    # header
    _rect(ax, x, top - _HEADER_H, w, _HEADER_H, _C_CLASS_HEADER_BG)
    ax.text(x + w / 2, top - _HEADER_H * 0.28, node.class_name,
            fontsize=9.0, color=_C_CLASS_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)
    ax.text(x + w / 2, top - _HEADER_H * 0.76, "<<circuit>>",
            fontsize=6.0, color=_C_STEREOTYPE_FG, ha="center", va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)

    cursor = top - _HEADER_H

    if node.direct_attributes:
        cursor = _section(ax, x, cursor, w, _C_SECTION_SCALAR, None,
                          node.direct_attributes, _C_BORDER_CLASS)

    if node.aggregation_attributes:
        cursor = _section(ax, x, cursor, w, _C_SECTION_AGG, "aggregations",
                          node.aggregation_attributes, _C_BORDER_CLASS)

    # circuit stats
    stats_h = _section_h(2)
    _rect(ax, x, cursor - stats_h, w, stats_h, _C_SECTION_CIRCUIT)
    _hline(ax, x, cursor, w, color=_C_BORDER_CLASS)
    ax.text(x + 0.14, cursor - _SEC_PAD_TOP * 0.6, "circuit",
            fontsize=5.5, color=_C_MUTED, va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)
    row_y = cursor - _SEC_PAD_TOP - _LH / 2
    for label, val in [("clusters", node.num_clusters), ("leaves", node.num_leaves)]:
        ax.text(x + 0.22, row_y, label,
                fontsize=6.5, color=_C_TEXT, va="center", ha="left",
                fontfamily="sans-serif", zorder=4)
        ax.text(x + w - 0.22, row_y, str(val),
                fontsize=6.5, color=_C_MUTED, va="center", ha="right",
                fontfamily="sans-serif", fontweight="bold", zorder=4)
        row_y -= _LH

    _outer_border(ax, x, y, w, h, _C_BORDER_CLASS)


def _draw_subobj_node(ax, node: _SubObjectNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    _rect(ax, x, top - _HEADER_H, w, _HEADER_H, _C_SUBOBJ_HEADER_BG)
    ax.text(x + w / 2, top - _HEADER_H / 2, node.sub_object_name,
            fontsize=8.0, color=_C_SUBOBJ_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)

    _section(ax, x, top - _HEADER_H, w, _C_SECTION_SCALAR, None,
             node.attributes, _C_BORDER_SUBOBJ)
    _outer_border(ax, x, y, w, h, _C_BORDER_SUBOBJ, lw=1.1)


def _draw_edt_node(ax, node: _EDTNode) -> None:
    x, y, w, h = node.x, node.y, node.width, node.height
    top = y + h

    _rect(ax, x, top - _HEADER_H, w, _HEADER_H, _C_EDT_HEADER_BG)
    ax.text(x + w / 2, top - _HEADER_H * 0.28, node.relation_name,
            fontsize=8.5, color=_C_EDT_HEADER_FG, ha="center", va="center",
            fontfamily="sans-serif", fontweight="bold", zorder=4)
    ax.text(x + w / 2, top - _HEADER_H * 0.76, "<<template>>",
            fontsize=6.0, color=_C_EDT_STEREOTYPE_FG, ha="center", va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4)

    cursor = top - _HEADER_H
    _section(ax, x, cursor, w, _C_SECTION_LATENT, "latent variables",
             node.latent_attributes, _C_BORDER_EDT)
    _outer_border(ax, x, y, w, h, _C_BORDER_EDT, lw=1.2)


# ── arrow drawing ─────────────────────────────────────────────────────────────


def _diamond(ax, cx: float, cy: float, color: str, size: float = 0.16) -> None:
    """Draw a filled UML diamond centred at (cx, cy)."""
    pts = np.array([
        [cx,            cy + size],
        [cx + size * 0.55, cy],
        [cx,            cy - size],
        [cx - size * 0.55, cy],
    ])
    ax.fill(pts[:, 0], pts[:, 1], color=color, zorder=5)


def _open_arrowhead(ax, tip_x: float, tip_y: float, color: str) -> None:
    """Draw an open arrowhead pointing upward at (tip_x, tip_y)."""
    ax.annotate(
        "", xy=(tip_x, tip_y + 0.01), xytext=(tip_x, tip_y + 0.35),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=1.1, mutation_scale=10),
        zorder=4,
    )


def _draw_elbow(ax, x1: float, y1: float, x2: float, y2: float,
                color: str) -> None:
    """Draw an L-shaped (or straight) connector from (x1,y1) to (x2,y2)."""
    if abs(x1 - x2) < 0.01:
        ax.plot([x1, x2], [y1, y2], color=color, linewidth=1.0, zorder=2)
    else:
        mid_y = (y1 + y2) / 2
        ax.plot([x1, x1, x2, x2], [y1, mid_y, mid_y, y2],
                color=color, linewidth=1.0, zorder=2, solid_capstyle="round")


def _draw_class_to_subobj_arrow(ax, parent: _ClassNode, child: _SubObjectNode) -> None:
    """Composition arrow: filled diamond on parent bottom, line to child top."""
    px = parent.x + parent.width / 2
    py = parent.y

    cx = child.x + child.width / 2
    cy = child.y + child.height

    _draw_elbow(ax, px, py, cx, cy, _C_ARROW_COMP)
    _diamond(ax, px, py, _C_ARROW_COMP)

    mid_y = (py + cy) / 2
    ax.text((px + cx) / 2 + 0.12, mid_y, "1",
            fontsize=7, color=_C_MULT, va="center", fontweight="bold",
            fontfamily="sans-serif", zorder=4)


def _draw_class_to_edt_arrow(ax, parent: _ClassNode, edt: _EDTNode) -> None:
    """Solid line from parent bottom to EDT top, with '1' multiplicity."""
    px = parent.x + parent.width / 2
    py = parent.y

    ex = edt.x + edt.width / 2
    ey = edt.y + edt.height

    _draw_elbow(ax, px, py, ex, ey, _C_ARROW_EDT)
    # no diamond here — just a plain association line
    ax.text((px + ex) / 2 + 0.12, (py + ey) / 2,
            "1", fontsize=7, color=_C_MULT, va="center",
            fontweight="bold", fontfamily="sans-serif", zorder=4)


def _draw_edt_to_child_arrow(ax, edt: _EDTNode, child: _ClassNode) -> None:
    """Open arrowhead from EDT bottom to child class top, with '*' multiplicity."""
    ex = edt.x + edt.width / 2
    ey = edt.y

    cx = child.x + child.width / 2
    cy = child.y + child.height

    _draw_elbow(ax, ex, ey, cx, cy, _C_ARROW_INST)
    _open_arrowhead(ax, cx, cy, _C_ARROW_INST)

    ax.text((ex + cx) / 2 + 0.12, (ey + cy) / 2,
            "*", fontsize=9, color=_C_MULT, va="center",
            fontweight="bold", fontfamily="sans-serif", zorder=4)


def _draw_all(ax, node: _AnyNode) -> None:
    """Recursively draw all nodes and their connecting arrows."""
    match node:
        case _SubObjectNode():
            _draw_subobj_node(ax, node)

        case _EDTNode():
            _draw_edt_node(ax, node)
            _draw_edt_to_child_arrow(ax, node, node.child)
            _draw_all(ax, node.child)

        case _ClassNode():
            _draw_class_node(ax, node)
            for sub in node.sub_objects:
                _draw_class_to_subobj_arrow(ax, node, sub)
                _draw_all(ax, sub)
            for edt in node.edt_children:
                _draw_class_to_edt_arrow(ax, node, edt)
                _draw_all(ax, edt)


# ── legend ────────────────────────────────────────────────────────────────────


def _draw_legend(ax, x: float, y: float) -> None:
    """Draw a compact type and node-kind legend."""
    gap = 2.2
    # node kinds
    kinds = [
        (_C_CLASS_HEADER_BG, "<<circuit>>  class"),
        (_C_SUBOBJ_HEADER_BG, "value object (sub-object)"),
        (_C_EDT_HEADER_BG, "<<template>>  EDT"),
    ]
    for i, (color, label) in enumerate(kinds):
        rx = x + i * gap * 1.5
        ax.add_patch(Rectangle((rx, y - 0.12), 0.28, 0.28,
                                facecolor=color, edgecolor="none", zorder=3))
        ax.text(rx + 0.35, y + 0.02, label,
                fontsize=7, color=_C_MUTED, va="center",
                fontfamily="sans-serif")

    # type colours
    type_x = x
    type_y = y - 0.52
    ax.text(type_x, type_y + 0.24, "Attribute types:",
            fontsize=7, color=_C_MUTED, va="center",
            fontfamily="sans-serif", fontweight="bold")
    offset = 0.0
    for type_name, short in _TYPE_SHORT.items():
        color = _TYPE_COLOR[type_name]
        ax.text(type_x + offset, type_y, f"{short}",
                fontsize=7, color=color, va="center",
                fontfamily="sans-serif", fontstyle="italic",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#F0F0F0",
                          edgecolor=color, linewidth=0.8))
        ax.text(type_x + offset + 0.32, type_y, f"= {type_name}  ",
                fontsize=7, color=_C_MUTED, va="center",
                fontfamily="sans-serif")
        offset += 2.2


# ── public API ────────────────────────────────────────────────────────────────


@dataclass
class RSPNPosterPlotter:
    """
    Renders a fitted :class:`~probabilistic_model.probabilistic_circuit.relational.rspn.RelationalProbabilisticCircuit`
    as a UML-style class diagram in SVG (or PNG / PDF).

    The diagram contains three node kinds:

    * **Class boxes** (dark header, ``<<circuit>>``) — one per RSPN class,
      with compartments for scalar attributes, aggregation statistics, and
      circuit summary (clusters / leaves).
    * **Sub-object boxes** (blue header) — one per nested value-object group,
      connected to its owning class by a composition arrow with diamond.
    * **Template boxes** (brown header, ``<<template>>``) — one per
      exchangeable-distribution template, sitting between the parent class
      and its child class, listing the latent conditioning variables.

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
        """Construct and return the matplotlib figure."""
        root = _build_class_node(self.rspn, set())
        _compute_heights(root)

        tree_w = _subtree_width(root)
        _place(root, tree_w / 2.0, 0.0)

        all_nodes = _collect_all(root)
        x0, y0, x1, y1 = _bounds(all_nodes)

        pad = self.padding
        fig_w = max((x1 - x0) + 2 * pad, 8.0)
        fig_h = (y1 - y0) + 2 * pad + 1.2   # bottom margin for legend

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=self.dpi)
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y0 - pad - 1.1, y1 + pad)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.patch.set_facecolor("white")

        ax.text(
            (x0 + x1) / 2, y1 + pad * 0.65,
            f"Relational Probabilistic Circuit  —  {self.rspn.class_.__name__}",
            fontsize=11, fontweight="bold", color=_C_CLASS_HEADER_BG,
            ha="center", va="center", fontfamily="sans-serif",
        )

        _draw_all(ax, root)
        _draw_legend(ax, x0, y0 - pad * 0.55)

        fig.tight_layout(pad=0.1)
        return fig

    def save(self, path: str) -> None:
        """
        Render and save to *path*.

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

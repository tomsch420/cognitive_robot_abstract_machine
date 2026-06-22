"""
UML-style class diagram explainer for :class:`RelationalProbabilisticCircuit`.

Renders a fitted RSPN as a class diagram where each class in the relational
hierarchy is a compartmented box (class name / scalar attributes /
aggregation statistics / circuit summary) connected by UML composition
arrows with multiplicity labels.  Output is a single SVG file.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import FancyArrowPatch, Rectangle, FancyBboxPatch
import matplotlib.patheffects as pe
import numpy as np

matplotlib.use("Agg")

from random_events.variable import Continuous, Symbolic, Integer

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import SumUnit

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
    )


# ── palette ──────────────────────────────────────────────────────────────────

_C_HEADER_BG: str = "#1B2631"        # dark navy — class name band
_C_HEADER_FG: str = "#FFFFFF"
_C_STEREOTYPE_FG: str = "#85C1E9"    # light blue for «circuit»
_C_SECTION_BG: str = "#EBF5FB"       # very light blue — scalar section
_C_AGG_BG: str = "#FEF9E7"           # light yellow — aggregation section
_C_CIRCUIT_BG: str = "#F9F9F9"       # near-white — circuit stats section
_C_BORDER: str = "#2C3E50"
_C_TEXT: str = "#17202A"
_C_MUTED: str = "#5D6D7E"
_C_ARROW: str = "#2C3E50"
_C_MULT: str = "#C0392B"             # red multiplicities
_C_REL_NAME: str = "#1A5276"         # dark blue relation name

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

# ── geometry ─────────────────────────────────────────────────────────────────

_BOX_W: float = 4.8          # card width
_LH: float = 0.32            # line height inside sections
_HEADER_H: float = 0.80      # class name + stereotype band
_SECTION_PAD_TOP: float = 0.18
_SECTION_PAD_BOT: float = 0.12
_COL_GAP: float = 1.4        # horizontal gap between sibling subtrees
_ROW_GAP: float = 2.6        # vertical gap between parent and child row


# ── data ─────────────────────────────────────────────────────────────────────


@dataclass
class _Attribute:
    """A single modelled variable shown in the class box."""

    name: str
    """Stripped display name (class prefix removed)."""

    type_name: str
    """One of Continuous / Integer / Symbolic."""


@dataclass
class _CardData:
    """All display information for one class node in the diagram."""

    class_name: str
    num_clusters: int
    num_leaves: int
    scalar_attributes: list[_Attribute]
    aggregation_attributes: list[_Attribute]
    relation_name: str | None
    """Field name of the incoming relation from the parent (None for root)."""

    children: list[_CardData] = field(default_factory=list)

    # ── placed by layout ──────────────────────────────────────────────────
    x: float = 0.0   # left edge
    y: float = 0.0   # bottom edge
    width: float = _BOX_W
    height: float = 0.0


# ── extraction ───────────────────────────────────────────────────────────────


def _strip_prefix(name: str, class_name: str) -> str:
    """Remove leading ``ClassName.`` or ``ClassNameAggregations.`` prefix."""
    for prefix in (f"{class_name}Aggregations.", f"{class_name}."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _var_type(variable) -> str:
    if isinstance(variable, Continuous):
        return "Continuous"
    if isinstance(variable, Symbolic):
        return "Symbolic"
    if isinstance(variable, Integer):
        return "Integer"
    return type(variable).__name__


def _extract(
    rspn: RelationalProbabilisticCircuit,
    relation_name: str | None,
    latent_names: set[str],
) -> _CardData:
    """
    Recursively build :class:`_CardData` from an RSPN node.

    :param rspn: The RSPN to summarise.
    :param relation_name: Incoming relation field name from parent.
    :param latent_names: Variable names treated as latent (conditioning
        context shared with the parent) that should not appear in the card.
    :return: A populated :class:`_CardData` tree.
    """
    pc = rspn.class_probabilistic_circuit
    class_name = rspn.class_.__name__

    num_clusters = sum(1 for n in pc.nodes() if isinstance(n, SumUnit))
    num_leaves = len(pc.leaves)

    agg_names: set[str] = set()
    if rspn.feature_extractor is not None:
        for variables in rspn.feature_extractor.exchangeable_features.values():
            agg_names.update(v._name_ for v in variables)

    scalar_attrs: list[_Attribute] = []
    agg_attrs: list[_Attribute] = []
    for var in pc.variables:
        if var.name in latent_names:
            continue
        display = _strip_prefix(var.name, class_name)
        attr = _Attribute(name=display, type_name=_var_type(var))
        if var.name in agg_names:
            agg_attrs.append(attr)
        else:
            scalar_attrs.append(attr)

    children: list[_CardData] = []
    for rel_name, template in rspn.exchangeable_distribution_templates.items():
        child_latents = {v.name for v in template.latent_variables}
        children.append(_extract(template.template_distribution, rel_name, child_latents))

    return _CardData(
        class_name=class_name,
        num_clusters=num_clusters,
        num_leaves=num_leaves,
        scalar_attributes=scalar_attrs,
        aggregation_attributes=agg_attrs,
        relation_name=relation_name,
        children=children,
    )


# ── layout ───────────────────────────────────────────────────────────────────


def _section_height(num_rows: int) -> float:
    """Height of one compartment with *num_rows* attribute lines."""
    return _SECTION_PAD_TOP + max(num_rows, 1) * _LH + _SECTION_PAD_BOT


def _compute_height(card: _CardData) -> float:
    """Total card height based on its content."""
    h = (
        _HEADER_H
        + _section_height(len(card.scalar_attributes))
        + (_section_height(len(card.aggregation_attributes)) if card.aggregation_attributes else 0.0)
        + _section_height(2)  # circuit stats: clusters + leaves
    )
    return h


def _subtree_width(card: _CardData) -> float:
    """Horizontal footprint of a card and all its descendants."""
    if not card.children:
        return card.width
    children_total = (
        sum(_subtree_width(c) for c in card.children)
        + _COL_GAP * (len(card.children) - 1)
    )
    return max(card.width, children_total)


def _place(card: _CardData, x_center: float, y_top: float) -> None:
    """Recursively assign positions; y_top is the top edge of the card."""
    card.height = _compute_height(card)
    card.x = x_center - card.width / 2.0
    card.y = y_top - card.height

    if not card.children:
        return

    total_children_w = (
        sum(_subtree_width(c) for c in card.children)
        + _COL_GAP * (len(card.children) - 1)
    )
    child_top = card.y - _ROW_GAP
    cursor_x = x_center - total_children_w / 2.0
    for child in card.children:
        sw = _subtree_width(child)
        _place(child, cursor_x + sw / 2.0, child_top)
        cursor_x += sw + _COL_GAP


def _collect(card: _CardData) -> list[_CardData]:
    result = [card]
    for child in card.children:
        result.extend(_collect(child))
    return result


def _bounds(cards: list[_CardData]) -> tuple[float, float, float, float]:
    x0 = min(c.x for c in cards)
    y0 = min(c.y for c in cards)
    x1 = max(c.x + c.width for c in cards)
    y1 = max(c.y + c.height for c in cards)
    return x0, y0, x1, y1


# ── drawing ───────────────────────────────────────────────────────────────────


def _rect(ax, x: float, y: float, w: float, h: float, facecolor: str, border: bool = True) -> None:
    """Draw a plain filled rectangle."""
    ax.add_patch(Rectangle(
        (x, y), w, h,
        facecolor=facecolor,
        edgecolor=_C_BORDER if border else "none",
        linewidth=0.8,
        zorder=2,
    ))


def _divider(ax, x: float, y: float, w: float) -> None:
    """Draw a thin horizontal divider line."""
    ax.plot([x, x + w], [y, y], color=_C_BORDER, linewidth=0.8, zorder=3)


def _attr_line(ax, x: float, y: float, attr: _Attribute, indent: float = 0.22) -> None:
    """Draw one ``name : type`` attribute line."""
    type_short = _TYPE_SHORT.get(attr.type_name, attr.type_name)
    type_color = _TYPE_COLOR.get(attr.type_name, _C_MUTED)
    ax.text(
        x + indent, y,
        attr.name,
        fontsize=6.5, color=_C_TEXT, va="center", ha="left",
        fontfamily="sans-serif", zorder=4,
    )
    ax.text(
        x + indent, y,
        f"{attr.name} :",
        fontsize=6.5, color=_C_TEXT, va="center", ha="left",
        fontfamily="sans-serif", zorder=4, alpha=0,  # invisible — just for width estimation
    )
    # draw " : type" after the name
    name_approx_width = len(attr.name) * 0.055  # rough character width in data units
    ax.text(
        x + indent + name_approx_width, y,
        f" : {type_short}",
        fontsize=6.5, color=type_color, va="center", ha="left",
        fontfamily="sans-serif", fontstyle="italic", zorder=4,
    )


def _draw_card(ax, card: _CardData) -> None:
    """Draw one UML-style class box."""
    x, y, w, h = card.x, card.y, card.width, card.height
    top = y + h

    # ── outer border (drawn last so it clips nothing) ─────────────────────
    ax.add_patch(Rectangle(
        (x, y), w, h,
        facecolor="none",
        edgecolor=_C_BORDER,
        linewidth=1.4,
        zorder=5,
    ))

    # ── header compartment ────────────────────────────────────────────────
    header_y = top - _HEADER_H
    _rect(ax, x, header_y, w, _HEADER_H, _C_HEADER_BG, border=False)
    # stereotype
    ax.text(
        x + w / 2, header_y + _HEADER_H * 0.72,
        "<<circuit>>",
        fontsize=6.0, color=_C_STEREOTYPE_FG, ha="center", va="center",
        fontfamily="sans-serif", fontstyle="italic", zorder=4,
    )
    # class name
    ax.text(
        x + w / 2, header_y + _HEADER_H * 0.30,
        card.class_name,
        fontsize=9.0, color=_C_HEADER_FG, ha="center", va="center",
        fontfamily="sans-serif", fontweight="bold", zorder=4,
    )

    cursor_y = header_y

    # ── scalar attributes compartment ─────────────────────────────────────
    scalar_h = _section_height(len(card.scalar_attributes))
    scalar_y = cursor_y - scalar_h
    _rect(ax, x, scalar_y, w, scalar_h, _C_SECTION_BG, border=False)
    _divider(ax, x, cursor_y, w)

    row_y = cursor_y - _SECTION_PAD_TOP - _LH / 2
    for attr in card.scalar_attributes:
        _attr_line(ax, x, row_y, attr)
        row_y -= _LH
    cursor_y = scalar_y

    # ── aggregation compartment (only if non-empty) ───────────────────────
    if card.aggregation_attributes:
        agg_h = _section_height(len(card.aggregation_attributes))
        agg_y = cursor_y - agg_h
        _rect(ax, x, agg_y, w, agg_h, _C_AGG_BG, border=False)
        _divider(ax, x, cursor_y, w)

        # section label
        ax.text(
            x + 0.14, cursor_y - _SECTION_PAD_TOP * 0.55,
            "aggregations",
            fontsize=5.5, color=_C_MUTED, va="center",
            fontfamily="sans-serif", fontstyle="italic", zorder=4,
        )

        row_y = cursor_y - _SECTION_PAD_TOP - _LH / 2
        for attr in card.aggregation_attributes:
            _attr_line(ax, x, row_y, attr)
            row_y -= _LH
        cursor_y = agg_y

    # ── circuit stats compartment ─────────────────────────────────────────
    stats_h = _section_height(2)
    stats_y = cursor_y - stats_h
    _rect(ax, x, stats_y, w, stats_h, _C_CIRCUIT_BG, border=False)
    _divider(ax, x, cursor_y, w)

    ax.text(
        x + 0.14, cursor_y - _SECTION_PAD_TOP * 0.55,
        "circuit",
        fontsize=5.5, color=_C_MUTED, va="center",
        fontfamily="sans-serif", fontstyle="italic", zorder=4,
    )

    stats_row_y = cursor_y - _SECTION_PAD_TOP - _LH / 2
    for label, value in [("clusters", card.num_clusters), ("leaves", card.num_leaves)]:
        ax.text(
            x + 0.22, stats_row_y,
            f"{label}",
            fontsize=6.5, color=_C_TEXT, va="center", ha="left",
            fontfamily="sans-serif", zorder=4,
        )
        ax.text(
            x + w - 0.22, stats_row_y,
            str(value),
            fontsize=6.5, color=_C_MUTED, va="center", ha="right",
            fontfamily="sans-serif", fontweight="bold", zorder=4,
        )
        stats_row_y -= _LH


def _diamond(ax, x: float, y: float, size: float = 0.18) -> None:
    """Draw a filled composition diamond centred at (x, y)."""
    pts = np.array([
        [x, y + size],          # top
        [x + size * 0.55, y],   # right
        [x, y - size],          # bottom
        [x - size * 0.55, y],   # left
    ])
    ax.fill(pts[:, 0], pts[:, 1], color=_C_ARROW, zorder=5)


def _draw_relation(ax, parent: _CardData, child: _CardData) -> None:
    """
    Draw a UML composition arrow from *parent* to *child*.

    Uses a filled diamond on the parent end (composition) and an open
    arrowhead on the child end, with ``1`` and ``*`` multiplicities and
    the relation field name on the line.
    """
    # anchor points: bottom-centre of parent, top-centre of child
    px = parent.x + parent.width / 2
    py = parent.y
    cx = child.x + child.width / 2
    cy = child.y + child.height

    # elbow: straight vertical segments joined by a horizontal if needed
    mid_y = (py + cy) / 2

    if abs(px - cx) < 0.01:
        # perfectly vertical — single straight line
        line_xs = [px, cx]
        line_ys = [py, cy]
    else:
        # L-shaped elbow
        line_xs = [px, px, cx, cx]
        line_ys = [py, mid_y, mid_y, cy]

    ax.plot(line_xs, line_ys, color=_C_ARROW, linewidth=1.1, zorder=3,
            solid_capstyle="round")

    # open arrowhead at child end (pointing up into the child top)
    ax.annotate(
        "", xy=(cx, cy + 0.01), xytext=(cx, cy + 0.35),
        arrowprops=dict(arrowstyle="-|>", color=_C_ARROW, lw=1.1,
                        mutation_scale=10),
        zorder=4,
    )

    # composition diamond at parent bottom
    _diamond(ax, px, py)

    # multiplicities
    offset_x = 0.12
    ax.text(px + offset_x, py - 0.22, "1",
            fontsize=7, color=_C_MULT, va="center", fontweight="bold",
            fontfamily="sans-serif", zorder=4)
    ax.text(cx + offset_x, cy + 0.22, "*",
            fontsize=9, color=_C_MULT, va="center", fontweight="bold",
            fontfamily="sans-serif", zorder=4)

    # relation field name on the elbow midpoint
    label = child.relation_name or ""
    label_x = (px + cx) / 2 + 0.15
    label_y = mid_y if abs(px - cx) >= 0.01 else (py + cy) / 2
    ax.text(label_x, label_y, label,
            fontsize=6.5, color=_C_REL_NAME, va="center",
            fontfamily="sans-serif", fontstyle="italic", fontweight="bold",
            zorder=4)


def _draw_tree(ax, card: _CardData) -> None:
    """Recursively draw all cards and relation arrows."""
    _draw_card(ax, card)
    for child in card.children:
        _draw_relation(ax, card, child)
        _draw_tree(ax, child)


# ── legend ────────────────────────────────────────────────────────────────────


def _draw_legend(ax, x: float, y: float) -> None:
    """Draw a compact type legend."""
    ax.text(x, y + 0.28, "Type notation:",
            fontsize=7, color=_C_MUTED, va="center",
            fontfamily="sans-serif", fontweight="bold")
    offset = 0.0
    for type_name, short in _TYPE_SHORT.items():
        color = _TYPE_COLOR[type_name]
        ax.text(x + offset, y, f"{short}",
                fontsize=7, color=color, va="center",
                fontfamily="sans-serif", fontstyle="italic",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#F0F0F0",
                          edgecolor=color, linewidth=0.8))
        ax.text(x + offset + 0.32, y, f"= {type_name}   ",
                fontsize=7, color=_C_MUTED, va="center",
                fontfamily="sans-serif")
        offset += 2.1


# ── public API ────────────────────────────────────────────────────────────────


@dataclass
class RSPNPosterPlotter:
    """
    Generates a UML-style class diagram for a fitted
    :class:`~probabilistic_model.probabilistic_circuit.relational.rspn.RelationalProbabilisticCircuit`.

    Each class in the relational hierarchy is drawn as a compartmented box:

    - **Header**: stereotype ``<<circuit>>`` and class name.
    - **Scalar attributes**: one ``name : type`` line per modelled feature.
    - **Aggregation statistics**: computed summaries over child collections.
    - **Circuit**: cluster count (mixture components) and leaf count.

    Classes are connected by UML composition arrows (filled diamond on the
    parent, open arrowhead on the child) labelled with the relation field
    name and ``1 / *`` multiplicities.

    .. note::
        The RSPN must be fitted before this plotter can be used.

    Example usage::

        plotter = RSPNPosterPlotter(rspn)
        plotter.save("rspn_diagram.svg")
    """

    rspn: RelationalProbabilisticCircuit
    """The fitted relational probabilistic circuit to visualise."""

    padding: float = 0.9
    """White-space margin around the diagram in figure units."""

    dpi: int = 150
    """Resolution used for raster formats (ignored for SVG)."""

    def _build(self) -> Figure:
        """Construct and return the matplotlib figure."""
        root = _extract(self.rspn, None, set())
        tree_w = _subtree_width(root)
        _place(root, tree_w / 2.0, 0.0)

        all_cards = _collect(root)
        x0, y0, x1, y1 = _bounds(all_cards)

        pad = self.padding
        fig_w = (x1 - x0) + 2 * pad
        fig_h = (y1 - y0) + 2 * pad + 0.9  # bottom margin for legend

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=self.dpi)
        ax.set_xlim(x0 - pad, x1 + pad)
        ax.set_ylim(y0 - pad - 0.8, y1 + pad)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.patch.set_facecolor("white")

        # diagram title
        ax.text(
            (x0 + x1) / 2, y1 + pad * 0.65,
            f"Relational Probabilistic Circuit  —  {self.rspn.class_.__name__}",
            fontsize=11, fontweight="bold", color=_C_HEADER_BG,
            ha="center", va="center", fontfamily="sans-serif",
        )

        _draw_tree(ax, root)
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
        fig.savefig(path, format=path.rsplit(".", 1)[-1], bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        plt.close(fig)

    def show(self) -> None:
        """Display the diagram interactively via :func:`matplotlib.pyplot.show`."""
        matplotlib.use("TkAgg")
        fig = self._build()
        plt.show()
        plt.close(fig)

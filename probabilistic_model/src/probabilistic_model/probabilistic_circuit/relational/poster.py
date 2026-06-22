"""
Poster-style SVG explainer for :class:`RelationalProbabilisticCircuit`.

Renders a high-level architectural overview of an ungrounded RSPN as a
tree of annotated class cards connected by relation arrows.  The output is
a single SVG file suitable for printing or embedding in documentation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import matplotlib
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
from matplotlib.figure import Figure

matplotlib.use("Agg")

from random_events.variable import Continuous, Symbolic, Integer

from probabilistic_model.probabilistic_circuit.rx.probabilistic_circuit import (
    SumUnit,
    ProductUnit,
    LeafUnit,
)

if TYPE_CHECKING:
    from probabilistic_model.probabilistic_circuit.relational.rspn import (
        RelationalProbabilisticCircuit,
    )


# ── visual constants ─────────────────────────────────────────────────────────

_CARD_WIDTH: float = 5.0
_CARD_MIN_HEIGHT: float = 3.5
_ROW_GAP: float = 2.8  # vertical gap between card rows
_COL_GAP: float = 1.2  # horizontal gap between sibling cards

_COLOR_HEADER: str = "#2C3E50"
_COLOR_SCALAR: str = "#EAF4FB"
_COLOR_AGG: str = "#FEF9E7"
_COLOR_BORDER: str = "#BDC3C7"
_COLOR_ARROW: str = "#7F8C8D"
_COLOR_RELATION_LABEL: str = "#E67E22"

_FONT_FAMILY: str = "monospace"

# type-tag colours
_TYPE_COLORS: dict[str, str] = {
    "Continuous": "#2ECC71",
    "Integer": "#3498DB",
    "Symbolic": "#9B59B6",
}
_TYPE_LABELS: dict[str, str] = {
    "Continuous": "cont",
    "Integer": "int",
    "Symbolic": "cat",
}


# ── data model ───────────────────────────────────────────────────────────────


@dataclass
class _CardData:
    """All display information for a single class card."""

    class_name: str
    """Human-readable class name."""

    num_clusters: int
    """Number of sum-node mixture components in the class circuit."""

    num_leaves: int
    """Number of leaf distributions."""

    scalar_variables: list[tuple[str, str]]
    """(display_name, variable_type_name) for each scalar variable."""

    aggregation_variables: list[tuple[str, str]]
    """(display_name, variable_type_name) for each aggregation statistic."""

    relation_name: str | None = None
    """Field name of the relation that produced this card (None for root)."""

    children: list[_CardData] = field(default_factory=list)
    """Child cards connected via exchangeable-distribution templates."""

    # layout fields populated during placement
    x: float = 0.0
    """Left edge of the card in figure coordinates."""

    y: float = 0.0
    """Bottom edge of the card in figure coordinates."""

    width: float = _CARD_WIDTH
    """Card width."""

    height: float = _CARD_MIN_HEIGHT
    """Card height, expanded to fit content."""


# ── extraction ───────────────────────────────────────────────────────────────


def _strip_class_prefix(name: str, class_name: str) -> str:
    """Remove the leading ``ClassName.`` or ``ClassNameAggregations.`` prefix."""
    for prefix in (f"{class_name}Aggregations.", f"{class_name}."):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def _variable_type_name(variable) -> str:
    if isinstance(variable, Continuous):
        return "Continuous"
    if isinstance(variable, Symbolic):
        return "Symbolic"
    if isinstance(variable, Integer):
        return "Integer"
    return type(variable).__name__


def _extract_card(
    rspn: RelationalProbabilisticCircuit,
    relation_name: str | None,
    latent_variable_names: set[str],
) -> _CardData:
    """
    Recursively extract display data from an RSPN node.

    :param rspn: The RSPN to summarise.
    :param relation_name: The field name linking this node to its parent.
    :param latent_variable_names: Variable names that are latent (shared with
        the parent circuit and used only for conditioning).
    :return: A fully populated :class:`_CardData` tree.
    """
    pc = rspn.class_probabilistic_circuit
    class_name = rspn.class_.__name__

    num_clusters = sum(1 for n in pc.nodes() if isinstance(n, SumUnit))
    num_leaves = len(pc.leaves)

    agg_names: set[str] = set()
    if rspn.feature_extractor is not None:
        for variables in rspn.feature_extractor.exchangeable_features.values():
            agg_names.update(v._name_ for v in variables)

    scalar_vars: list[tuple[str, str]] = []
    agg_vars: list[tuple[str, str]] = []
    for var in pc.variables:
        if var.name in latent_variable_names:
            continue
        display = _strip_class_prefix(var.name, class_name)
        type_name = _variable_type_name(var)
        if var.name in agg_names:
            agg_vars.append((display, type_name))
        else:
            scalar_vars.append((display, type_name))

    children: list[_CardData] = []
    for rel_name, template in rspn.exchangeable_distribution_templates.items():
        child_latent_names = {v.name for v in template.latent_variables}
        child_card = _extract_card(
            template.template_distribution,
            rel_name,
            child_latent_names,
        )
        children.append(child_card)

    return _CardData(
        class_name=class_name,
        num_clusters=num_clusters,
        num_leaves=num_leaves,
        scalar_variables=scalar_vars,
        aggregation_variables=agg_vars,
        relation_name=relation_name,
        children=children,
    )


# ── layout ───────────────────────────────────────────────────────────────────

_LINE_HEIGHT: float = 0.30
_HEADER_HEIGHT: float = 1.10
_SECTION_TITLE_HEIGHT: float = 0.32
_BOTTOM_PAD: float = 0.25


def _compute_card_height(card: _CardData) -> float:
    """Compute the height needed to display all rows in a card."""
    num_rows = max(len(card.scalar_variables), 1) + max(len(card.aggregation_variables), 0)
    content_height = (
        _SECTION_TITLE_HEIGHT  # "Scalar features" header
        + num_rows * _LINE_HEIGHT
        + (_SECTION_TITLE_HEIGHT if card.aggregation_variables else 0.0)
        + _BOTTOM_PAD
    )
    return max(_CARD_MIN_HEIGHT, _HEADER_HEIGHT + content_height)


def _subtree_width(card: _CardData) -> float:
    """Total horizontal space required by a card and all its descendants."""
    if not card.children:
        return card.width
    children_total = sum(_subtree_width(c) for c in card.children) + _COL_GAP * (
        len(card.children) - 1
    )
    return max(card.width, children_total)


def _place_cards(card: _CardData, x_center: float, y_bottom: float) -> None:
    """Recursively assign (x, y) positions to every card in the tree."""
    card.height = _compute_card_height(card)
    card.x = x_center - card.width / 2.0
    card.y = y_bottom

    if not card.children:
        return

    total_children_width = sum(_subtree_width(c) for c in card.children) + _COL_GAP * (
        len(card.children) - 1
    )
    child_y = y_bottom - card.height - _ROW_GAP
    cursor_x = x_center - total_children_width / 2.0
    for child in card.children:
        child_sw = _subtree_width(child)
        child_center = cursor_x + child_sw / 2.0
        _place_cards(child, child_center, child_y)
        cursor_x += child_sw + _COL_GAP


def _total_tree_height(card: _CardData) -> float:
    card.height = _compute_card_height(card)
    if not card.children:
        return card.height
    return card.height + _ROW_GAP + max(_total_tree_height(c) for c in card.children)


# ── drawing ──────────────────────────────────────────────────────────────────


def _draw_type_badge(ax, x: float, y: float, type_name: str) -> None:
    """Draw a small coloured type pill at (x, y)."""
    color = _TYPE_COLORS.get(type_name, "#95A5A6")
    label = _TYPE_LABELS.get(type_name, type_name[:3])
    ax.text(
        x,
        y,
        f" {label} ",
        fontsize=5.5,
        fontfamily=_FONT_FAMILY,
        color="white",
        va="center",
        ha="left",
        bbox=dict(boxstyle="round,pad=0.15", facecolor=color, edgecolor="none"),
    )


def _draw_card(ax, card: _CardData) -> None:
    """Draw a single class card onto *ax*."""
    x, y, w, h = card.x, card.y, card.width, card.height

    # outer box
    box = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.05",
        linewidth=1.2,
        edgecolor=_COLOR_BORDER,
        facecolor="white",
        zorder=2,
    )
    ax.add_patch(box)

    # header band
    header = FancyBboxPatch(
        (x, y + h - _HEADER_HEIGHT),
        w,
        _HEADER_HEIGHT,
        boxstyle="round,pad=0.05",
        linewidth=0,
        facecolor=_COLOR_HEADER,
        zorder=3,
    )
    ax.add_patch(header)

    # class name
    ax.text(
        x + w / 2,
        y + h - _HEADER_HEIGHT / 2 - 0.08,
        card.class_name,
        fontsize=9,
        fontweight="bold",
        color="white",
        ha="center",
        va="center",
        fontfamily=_FONT_FAMILY,
        zorder=4,
    )

    # circuit summary row
    summary = (
        f"vars: {card.num_leaves}   "
        f"clusters: {card.num_clusters}"
    )
    ax.text(
        x + w / 2,
        y + h - _HEADER_HEIGHT + 0.22,
        summary,
        fontsize=6.0,
        color="#BDC3C7",
        ha="center",
        va="center",
        fontfamily=_FONT_FAMILY,
        zorder=4,
    )

    # divider
    ax.plot(
        [x + 0.15, x + w - 0.15],
        [y + h - _HEADER_HEIGHT, y + h - _HEADER_HEIGHT],
        color=_COLOR_BORDER,
        linewidth=0.5,
        zorder=4,
    )

    cursor_y = y + h - _HEADER_HEIGHT - _SECTION_TITLE_HEIGHT * 0.9

    # ── scalar features ───────────────────────────────────────────────────
    ax.text(
        x + 0.18,
        cursor_y,
        "Scalar features",
        fontsize=6.5,
        fontweight="bold",
        color="#566573",
        va="center",
        zorder=4,
    )
    cursor_y -= _LINE_HEIGHT * 0.7

    col_mid = x + w / 2 + 0.05
    for display_name, type_name in card.scalar_variables:
        ax.text(
            x + 0.25,
            cursor_y,
            f"• {display_name}",
            fontsize=6.0,
            color="#2C3E50",
            va="center",
            fontfamily=_FONT_FAMILY,
            zorder=4,
        )
        _draw_type_badge(ax, col_mid, cursor_y, type_name)
        cursor_y -= _LINE_HEIGHT

    # ── aggregation statistics ─────────────────────────────────────────────
    if card.aggregation_variables:
        cursor_y -= 0.05
        ax.text(
            x + 0.18,
            cursor_y,
            "Aggregation statistics",
            fontsize=6.5,
            fontweight="bold",
            color="#566573",
            va="center",
            zorder=4,
        )
        cursor_y -= _LINE_HEIGHT * 0.7

        for display_name, type_name in card.aggregation_variables:
            ax.text(
                x + 0.25,
                cursor_y,
                f"• {display_name}",
                fontsize=6.0,
                color="#2C3E50",
                va="center",
                fontfamily=_FONT_FAMILY,
                zorder=4,
            )
            _draw_type_badge(ax, col_mid, cursor_y, type_name)
            cursor_y -= _LINE_HEIGHT


def _draw_arrow(ax, parent: _CardData, child: _CardData) -> None:
    """Draw a labeled arrow from the bottom of *parent* to the top of *child*."""
    px = parent.x + parent.width / 2
    py = parent.y  # bottom of parent

    cx = child.x + child.width / 2
    cy = child.y + child.height  # top of child

    arrow = FancyArrowPatch(
        (px, py),
        (cx, cy),
        arrowstyle="-|>",
        color=_COLOR_ARROW,
        linewidth=1.2,
        mutation_scale=10,
        connectionstyle="arc3,rad=0.0",
        zorder=1,
    )
    ax.add_patch(arrow)

    mid_x = (px + cx) / 2
    mid_y = (py + cy) / 2

    label = child.relation_name or ""
    ax.text(
        mid_x + 0.15,
        mid_y,
        f"{label}  1:N",
        fontsize=6.5,
        color=_COLOR_RELATION_LABEL,
        va="center",
        fontweight="bold",
        fontfamily=_FONT_FAMILY,
        zorder=4,
    )


def _draw_tree(ax, card: _CardData) -> None:
    """Recursively draw all cards and arrows."""
    _draw_card(ax, card)
    for child in card.children:
        _draw_arrow(ax, card, child)
        _draw_tree(ax, child)


def _collect_all(card: _CardData) -> list[_CardData]:
    result = [card]
    for child in card.children:
        result.extend(_collect_all(child))
    return result


def _figure_bounds(cards: list[_CardData]) -> tuple[float, float, float, float]:
    """Return (x_min, y_min, x_max, y_max) across all cards."""
    xs = [c.x for c in cards]
    ys = [c.y for c in cards]
    x2s = [c.x + c.width for c in cards]
    y2s = [c.y + c.height for c in cards]
    return min(xs), min(ys), max(x2s), max(y2s)


# ── legend ───────────────────────────────────────────────────────────────────


def _draw_legend(ax, x: float, y: float) -> None:
    """Draw a small type-badge legend at (x, y)."""
    ax.text(x, y + 0.3, "Variable types", fontsize=7, fontweight="bold", color="#566573")
    offset = 0.0
    for type_name, label in _TYPE_LABELS.items():
        color = _TYPE_COLORS[type_name]
        ax.text(
            x + offset,
            y,
            f" {label} ",
            fontsize=6,
            color="white",
            va="center",
            bbox=dict(boxstyle="round,pad=0.2", facecolor=color, edgecolor="none"),
        )
        ax.text(x + offset + 0.35, y, f"= {type_name}", fontsize=6, va="center", color="#2C3E50")
        offset += 1.8


# ── public API ───────────────────────────────────────────────────────────────


@dataclass
class RSPNPosterPlotter:
    """
    Generates a poster-style SVG diagram for a fitted
    :class:`~probabilistic_model.probabilistic_circuit.relational.rspn.RelationalProbabilisticCircuit`.

    The diagram shows each class in the relational hierarchy as an annotated
    card listing its scalar features, aggregation statistics, and circuit
    summary (variable count, cluster count).  Cards are connected by labeled
    arrows that indicate the relation field name and ``1:N`` multiplicity.

    .. note::
        The RSPN must be fitted (i.e. :meth:`fit` must have been called)
        before this plotter can be used.

    Example usage::

        plotter = RSPNPosterPlotter(rspn)
        plotter.save("rspn_poster.svg")
    """

    rspn: RelationalProbabilisticCircuit
    """The fitted relational probabilistic circuit to visualise."""

    padding: float = 0.8
    """White-space margin around the diagram in figure units."""

    dpi: int = 150
    """Resolution used when rasterising (ignored for SVG output)."""

    def _build(self) -> Figure:
        """Construct and return the matplotlib figure."""
        root_card = _extract_card(self.rspn, None, set())
        tree_w = _subtree_width(root_card)
        tree_h = _total_tree_height(root_card)

        _place_cards(root_card, tree_w / 2.0, 0.0)

        all_cards = _collect_all(root_card)
        x_min, y_min, x_max, y_max = _figure_bounds(all_cards)

        pad = self.padding
        fig_w = (x_max - x_min) + 2 * pad
        fig_h = (y_max - y_min) + 2 * pad + 1.0  # +1 for legend

        fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=self.dpi)
        ax.set_xlim(x_min - pad, x_max + pad)
        ax.set_ylim(y_min - pad - 0.9, y_max + pad)
        ax.set_aspect("equal")
        ax.axis("off")
        fig.patch.set_facecolor("#F8F9FA")

        # title
        ax.text(
            (x_min + x_max) / 2,
            y_max + pad * 0.6,
            f"Relational Probabilistic Circuit — {self.rspn.class_.__name__}",
            fontsize=12,
            fontweight="bold",
            color=_COLOR_HEADER,
            ha="center",
            va="center",
        )

        _draw_tree(ax, root_card)
        _draw_legend(ax, x_min, y_min - pad * 0.5)

        fig.tight_layout(pad=0.2)
        return fig

    def save(self, path: str) -> None:
        """
        Render and save the poster to *path*.

        The output format is inferred from the file extension.  Use ``.svg``
        for a lossless vector graphic, or ``.png`` / ``.pdf`` for raster /
        print formats.

        :param path: Destination file path, e.g. ``"rspn_poster.svg"``.
        """
        fig = self._build()
        fig.savefig(path, format=path.rsplit(".", 1)[-1], bbox_inches="tight")
        plt.close(fig)

    def show(self) -> None:
        """
        Render and display the poster interactively.

        Calls :func:`matplotlib.pyplot.show`; behaviour depends on the
        active matplotlib backend.
        """
        matplotlib.use("TkAgg")
        fig = self._build()
        plt.show()
        plt.close(fig)

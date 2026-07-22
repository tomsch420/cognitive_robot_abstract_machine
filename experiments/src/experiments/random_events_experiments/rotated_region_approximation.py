"""
This module measures what it costs a product algebra to represent a placement region
that is axis aligned in some other frame.

RoboCasa states where an object may go as a rectangle that is axis aligned in a
fixture's frame and then rigidly transformed by that fixture's yaw. An
:class:`~random_events.product_algebra.Event` over the world axes can only hold unions
of axis aligned boxes, so such a region is representable exactly only when the yaw is a
multiple of a quarter turn. Everything else has to be approximated by a staircase of
boxes.

Two approximations are measured, because a robot needs both guarantees for different
reasons. An inner approximation stays inside the region, so every sampled placement is
feasible but part of the region is lost. An outer approximation covers the region, so no
feasible placement is excluded but samples may fall outside it. The gap between them is
the price of the frame mismatch, in simple sets.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

import plotly.graph_objects as go
from typing_extensions import Iterator

from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    TypstRenderer,
)
from random_events.interval import closed, closed_open
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Continuous

QUARTER_TURN = math.pi / 2
"""
Yaw period after which a rectangle is axis aligned again, so only yaws below it matter.
"""

AXIS_ALIGNMENT_TOLERANCE = 1e-9
"""
Yaw distance from a quarter turn below which a region counts as axis aligned.

Multiples of a quarter turn are not exactly representable in binary floating point, and
the inscribed box of a region collapses discontinuously once the region is rotated at
all, so a region a fraction of a nanoradian off axis would otherwise be measured as
unrepresentable.
"""

PLACEMENT_X = Continuous("placement_x")
"""
World x coordinate of a placement, as the product algebra sees it.
"""

PLACEMENT_Y = Continuous("placement_y")
"""
World y coordinate of a placement, as the product algebra sees it.
"""


class ApproximationDirection(enum.Enum):
    """
    Which side of the true region a staircase of axis aligned boxes is allowed to fall
    on.

    Every member's value describes the guarantee it gives a robot sampling from it.
    """

    INNER = (
        "Stays inside the region, so every sampled placement is feasible while part of "
        "the region is lost."
    )
    OUTER = (
        "Covers the region, so no feasible placement is excluded while some samples "
        "fall outside it."
    )

    @property
    def description(self) -> str:
        """:return: The guarantee this direction gives."""
        return self.value


@dataclass(frozen=True)
class Box:
    """
    An axis aligned box in world coordinates, the only region a single simple event
    represents exactly.
    """

    x_lower: float
    """
    Lower bound along the world x axis.
    """

    x_upper: float
    """
    Upper bound along the world x axis.
    """

    y_lower: float
    """
    Lower bound along the world y axis.
    """

    y_upper: float
    """
    Upper bound along the world y axis.
    """

    @property
    def area(self) -> float:
        """:return: The area covered, zero for a box that collapsed to a line."""
        return max(0.0, self.x_upper - self.x_lower) * max(
            0.0, self.y_upper - self.y_lower
        )

    @property
    def is_empty(self) -> bool:
        """:return: Whether the box covers no area."""
        return self.area <= 0.0

    def to_simple_event(self) -> SimpleEvent:
        """
        :return: The simple event covering exactly this box, half open along x so that
            boxes of neighbouring strips stay disjoint.
        """
        return SimpleEvent.from_data(
            {
                PLACEMENT_X: closed_open(self.x_lower, self.x_upper),
                PLACEMENT_Y: closed(self.y_lower, self.y_upper),
            }
        )


@dataclass(frozen=True)
class RotatedRectangle:
    """
    A placement region that is axis aligned in a fixture's frame and rotated by that
    fixture's yaw in the frame the event is built in.
    """

    half_width: float
    """
    Half extent along the fixture's own x axis.
    """

    half_depth: float
    """
    Half extent along the fixture's own y axis.
    """

    yaw: float
    """
    Rotation of the fixture frame against the frame the event is built in.
    """

    @property
    def area(self) -> float:
        """:return: The area of the region, which rotation leaves unchanged."""
        return 4.0 * self.half_width * self.half_depth

    @property
    def rotation(self) -> tuple[float, float]:
        """
        :return: Cosine and sine of the yaw, taken exactly at multiples of a quarter
            turn so that a region meant to be axis aligned is measured as such.
        """
        quarter_turns = round(self.yaw / QUARTER_TURN)
        if abs(self.yaw - quarter_turns * QUARTER_TURN) >= AXIS_ALIGNMENT_TOLERANCE:
            return math.cos(self.yaw), math.sin(self.yaw)
        cosines = (1.0, 0.0, -1.0, 0.0)
        sines = (0.0, 1.0, 0.0, -1.0)
        return cosines[quarter_turns % 4], sines[quarter_turns % 4]

    @property
    def corners(self) -> list[tuple[float, float]]:
        """:return: The four corners in world coordinates, in order around the
        region."""
        cosine, sine = self.rotation
        local = [
            (self.half_width, self.half_depth),
            (-self.half_width, self.half_depth),
            (-self.half_width, -self.half_depth),
            (self.half_width, -self.half_depth),
        ]
        return [
            (x * cosine - y * sine, x * sine + y * cosine) for x, y in local
        ]

    @property
    def edges(self) -> Iterator[tuple[tuple[float, float], tuple[float, float]]]:
        """:return: The four edges as corner pairs."""
        corners = self.corners
        return zip(corners, corners[1:] + corners[:1])

    @property
    def horizontal_bounds(self) -> tuple[float, float]:
        """:return: The smallest and largest world x coordinate the region reaches."""
        abscissae = [corner[0] for corner in self.corners]
        return min(abscissae), max(abscissae)

    def vertical_span_at(self, abscissa: float) -> tuple[float, float] | None:
        """
        Intersect the region with a vertical line.

        :param abscissa: World x coordinate of the line.
        :return: The lowest and highest world y the region reaches there, or ``None``
            when the line misses the region.
        """
        ordinates = []
        for (start_x, start_y), (end_x, end_y) in self.edges:
            if start_x == abscissa:
                ordinates.append(start_y)
                continue
            if (start_x - abscissa) * (end_x - abscissa) > 0 or start_x == end_x:
                continue
            ratio = (abscissa - start_x) / (end_x - start_x)
            ordinates.append(start_y + ratio * (end_y - start_y))
        if not ordinates:
            return None
        return min(ordinates), max(ordinates)

    def corner_abscissae_between(self, lower: float, upper: float) -> list[float]:
        """
        :param lower: Lower world x bound of a strip.
        :param upper: Upper world x bound of a strip.
        :return: World x coordinates of corners lying strictly inside the strip, where
            the region's outline changes direction.
        """
        return [
            corner[0] for corner in self.corners if lower < corner[0] < upper
        ]


@dataclass
class StripDecomposition:
    """
    Approximates a rotated rectangle by a union of axis aligned boxes, one per vertical
    strip of equal width.

    Equal width strips are the decomposition a robot can build without knowing the
    region's shape in advance, which makes the resulting counts an honest price rather
    than a best case from a tuned decomposition.

    ..warning::
        The inner direction is pessimistic at small budgets. A box must fit under the
        region's outline across its whole strip, and the outline pinches to a point at
        the extreme corners, so a single strip spanning the region yields no area at
        all. That is a property of this decomposition, not a limit of the product
        algebra: a decomposition free to place narrower strips inside the region does
        far better with the same number of simple sets.
    """

    rectangle: RotatedRectangle
    """
    The region being approximated.
    """

    number_of_simple_sets: int
    """
    How many strips, and therefore boxes, the approximation is allowed to use.
    """

    direction: ApproximationDirection
    """
    Which side of the region the boxes are allowed to fall on.
    """

    def __post_init__(self):
        assert (
            self.number_of_simple_sets > 0
        ), "A region cannot be approximated by zero boxes."

    @property
    def strip_bounds(self) -> list[float]:
        """:return: The world x coordinates separating the strips."""
        lower, upper = self.rectangle.horizontal_bounds
        width = (upper - lower) / self.number_of_simple_sets
        return [lower + index * width for index in range(self.number_of_simple_sets + 1)]

    @property
    def boxes(self) -> list[Box]:
        """:return: The non empty boxes making up the approximation."""
        bounds = self.strip_bounds
        boxes = [
            self.box_for_strip(lower, upper)
            for lower, upper in zip(bounds, bounds[1:])
        ]
        return [box for box in boxes if not box.is_empty]

    def box_for_strip(self, lower: float, upper: float) -> Box:
        """
        :param lower: Lower world x bound of the strip.
        :param upper: Upper world x bound of the strip.
        :return: The box this strip contributes to the approximation.
        """
        if self.direction is ApproximationDirection.INNER:
            return self.inner_box_for_strip(lower, upper)
        return self.outer_box_for_strip(lower, upper)

    def inner_box_for_strip(self, lower: float, upper: float) -> Box:
        """
        The region's lower outline is convex and its upper outline concave, so the
        largest box fitting inside a strip is bounded by the outline at the strip's
        edges.

        :param lower: Lower world x bound of the strip.
        :param upper: Upper world x bound of the strip.
        :return: The largest box contained in the region over this strip.
        """
        spans = [
            self.rectangle.vertical_span_at(abscissa) for abscissa in (lower, upper)
        ]
        if any(span is None for span in spans):
            return Box(lower, upper, 0.0, 0.0)
        return Box(
            lower,
            upper,
            max(span[0] for span in spans),
            min(span[1] for span in spans),
        )

    def outer_box_for_strip(self, lower: float, upper: float) -> Box:
        """
        :param lower: Lower world x bound of the strip.
        :param upper: Upper world x bound of the strip.
        :return: The smallest box containing every part of the region over this strip.
        """
        abscissae = [lower, upper] + self.rectangle.corner_abscissae_between(
            lower, upper
        )
        spans = [
            span
            for span in (
                self.rectangle.vertical_span_at(abscissa) for abscissa in abscissae
            )
            if span is not None
        ]
        if not spans:
            return Box(lower, upper, 0.0, 0.0)
        return Box(
            lower,
            upper,
            min(span[0] for span in spans),
            max(span[1] for span in spans),
        )

    @property
    def area(self) -> float:
        """:return: The area the approximation covers."""
        return sum(box.area for box in self.boxes)

    @property
    def event(self) -> Event:
        """:return: The approximation as a product algebra event."""
        return Event.from_simple_sets(*[box.to_simple_event() for box in self.boxes])


@dataclass
class MeasuredEvent:
    """
    A product algebra event over the placement axes whose area can be measured, used to
    confirm that a decomposition really produces the region it claims to.
    """

    event: Event
    """
    The event to measure.
    """

    @property
    def area(self) -> float:
        """:return: The area covered, summed over disjoint simple sets."""
        return sum(
            self.extent(simple_event, PLACEMENT_X)
            * self.extent(simple_event, PLACEMENT_Y)
            for simple_event in self.event.make_disjoint().simple_sets
        )

    @staticmethod
    def extent(simple_event: SimpleEvent, variable: Continuous) -> float:
        """
        :param simple_event: The simple event to read.
        :param variable: The axis to measure along.
        :return: The total length the simple event assigns to the axis.
        """
        return sum(
            interval.upper - interval.lower
            for interval in simple_event[variable].simple_sets
        )


@dataclass
class ApproximationQuality(ExperimentResult):
    """
    How well a fixed number of simple sets represents a region at a given yaw.
    """

    yaw_degrees: float
    """
    Rotation of the region against the world axes, in degrees.
    """

    number_of_simple_sets: int
    """
    Simple sets the approximation was allowed to use.
    """

    retained_fraction: float
    """
    Share of the region an inner approximation keeps, so the share of feasible
    placements still reachable.
    """

    feasible_fraction: float
    """
    Share of an outer approximation that is truly in the region, so the share of samples
    that do not have to be rejected.
    """

    @classmethod
    def measure(
        cls, rectangle: RotatedRectangle, number_of_simple_sets: int
    ) -> ApproximationQuality:
        """
        :param rectangle: The region to approximate.
        :param number_of_simple_sets: Simple sets the approximation may use.
        :return: The quality both approximation directions reach.
        """
        inner = StripDecomposition(
            rectangle, number_of_simple_sets, ApproximationDirection.INNER
        )
        outer = StripDecomposition(
            rectangle, number_of_simple_sets, ApproximationDirection.OUTER
        )
        return cls(
            yaw_degrees=round(math.degrees(rectangle.yaw), 1),
            number_of_simple_sets=number_of_simple_sets,
            retained_fraction=round(inner.area / rectangle.area, 4),
            feasible_fraction=round(rectangle.area / outer.area, 4),
        )


@dataclass
class SimpleSetRequirement(ExperimentResult):
    """
    How many simple sets a yaw needs before both approximations reach a target quality.
    """

    yaw_degrees: float
    """
    Rotation of the region against the world axes, in degrees.
    """

    target_quality: float
    """
    Quality both approximations had to reach.
    """

    simple_sets_needed: int
    """
    Smallest number of simple sets reaching the target, or the search limit when the
    target was never reached.
    """

    reached_target: bool
    """
    Whether the target was reached within the search limit.
    """


@dataclass
class RotationPriceExperiment:
    """
    Measures the price of a frame mismatch across yaws and simple set budgets.
    """

    rectangle_half_extents: tuple[float, float] = (0.2, 0.15)
    """
    Half extents of the region being approximated, in metres.
    """

    yaws_in_degrees: tuple[float, ...] = (0.0, 5.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0)
    """
    Rotations to measure, spanning a quarter turn since the region repeats after that.
    """

    simple_set_budgets: tuple[int, ...] = (1, 2, 4, 8, 16, 32, 64, 128)
    """
    Simple set counts to measure at.
    """

    search_limit: int = 4096
    """
    Largest number of simple sets considered when searching for a target quality.
    """

    def rectangle_at(self, yaw_in_degrees: float) -> RotatedRectangle:
        """
        :param yaw_in_degrees: The rotation to build the region at.
        :return: The region rotated by that yaw.
        """
        half_width, half_depth = self.rectangle_half_extents
        return RotatedRectangle(half_width, half_depth, math.radians(yaw_in_degrees))

    def quality_table(self) -> ExperimentsTable:
        """:return: Approximation quality for every yaw and simple set budget."""
        return ExperimentsTable(
            [
                ApproximationQuality.measure(
                    self.rectangle_at(yaw), number_of_simple_sets
                )
                for yaw in self.yaws_in_degrees
                for number_of_simple_sets in self.simple_set_budgets
            ]
        )

    def requirement_table(self, target_quality: float) -> ExperimentsTable:
        """
        :param target_quality: Quality both approximations must reach.
        :return: The simple sets each yaw needs to reach the target.
        """
        return ExperimentsTable(
            [
                self.requirement_at(yaw, target_quality)
                for yaw in self.yaws_in_degrees
            ]
        )

    def requirement_at(
        self, yaw_in_degrees: float, target_quality: float
    ) -> SimpleSetRequirement:
        """
        Search increasing simple set counts until both approximations reach the target.

        :param yaw_in_degrees: The rotation to measure at.
        :param target_quality: Quality both approximations must reach.
        :return: The smallest sufficient number of simple sets.
        """
        rectangle = self.rectangle_at(yaw_in_degrees)
        number_of_simple_sets = 1
        while number_of_simple_sets <= self.search_limit:
            quality = ApproximationQuality.measure(rectangle, number_of_simple_sets)
            if (
                quality.retained_fraction >= target_quality
                and quality.feasible_fraction >= target_quality
            ):
                return SimpleSetRequirement(
                    yaw_degrees=round(yaw_in_degrees, 1),
                    target_quality=target_quality,
                    simple_sets_needed=number_of_simple_sets,
                    reached_target=True,
                )
            number_of_simple_sets *= 2
        return SimpleSetRequirement(
            yaw_degrees=round(yaw_in_degrees, 1),
            target_quality=target_quality,
            simple_sets_needed=self.search_limit,
            reached_target=False,
        )


APPROXIMATION_COLORS = {
    ApproximationDirection.INNER: "#2a78d6",
    ApproximationDirection.OUTER: "#eb6834",
}
"""
Fixed categorical color per approximation direction.
"""


def plot_rotation_price(experiment: RotationPriceExperiment) -> go.Figure:
    """
    Show how both approximations improve as the simple set budget grows.

    One trace per yaw and direction, with the simple set budget on a log x axis since
    budgets double, and quality on a linear y axis since it is a fraction.

    :param experiment: The experiment whose measurements are plotted.
    :return: A figure comparing every measured yaw.
    """
    figure = go.Figure()
    for yaw in experiment.yaws_in_degrees:
        rectangle = experiment.rectangle_at(yaw)
        qualities = [
            ApproximationQuality.measure(rectangle, budget)
            for budget in experiment.simple_set_budgets
        ]
        for direction, values in (
            (
                ApproximationDirection.INNER,
                [quality.retained_fraction for quality in qualities],
            ),
            (
                ApproximationDirection.OUTER,
                [quality.feasible_fraction for quality in qualities],
            ),
        ):
            figure.add_trace(
                go.Scatter(
                    x=list(experiment.simple_set_budgets),
                    y=values,
                    mode="lines+markers",
                    name=f"{yaw:.0f}° {direction.name.lower()}",
                    line=dict(color=APPROXIMATION_COLORS[direction], width=2),
                    opacity=0.35 + 0.65 * yaw / max(experiment.yaws_in_degrees),
                    hovertemplate=(
                        f"Yaw: {yaw:.0f}°<br>"
                        f"Direction: {direction.name.lower()}<br>"
                        "Simple sets: %{x}<br>"
                        "Quality: %{y:.3f}<extra></extra>"
                    ),
                )
            )

    figure.update_layout(
        title="Price of Representing a Rotated Placement Region",
        xaxis_title="Simple Sets in the Approximation",
        yaxis_title="Fraction of the Region Represented Correctly",
        xaxis_type="log",
        plot_bgcolor="#fcfcfb",
        paper_bgcolor="#fcfcfb",
        font=dict(color="#0b0b0b"),
    )
    figure.update_xaxes(gridcolor="#e1e0d9", zerolinecolor="#c3c2b7")
    figure.update_yaxes(gridcolor="#e1e0d9", zerolinecolor="#c3c2b7", range=[0, 1.02])
    return figure


def main():
    experiment = RotationPriceExperiment()

    print(
        TypstRenderer(experiment.quality_table()).render_figure(
            "Quality reached when a rotated placement region is represented by a fixed "
            "number of axis aligned boxes. The retained fraction is how much of the "
            "region an inner approximation keeps; the feasible fraction is how much of "
            "an outer approximation really lies in the region."
        )
    )
    for target_quality in (0.95, 0.99):
        print()
        print(
            TypstRenderer(experiment.requirement_table(target_quality)).render_figure(
                f"Simple sets needed before both approximations of a rotated placement "
                f"region reach a quality of {target_quality}."
            )
        )


if __name__ == "__main__":
    main()

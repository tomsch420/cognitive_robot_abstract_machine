"""
This module quantifies what it costs a product algebra to represent a placement whose
admissible positions depend on the object's orientation.

A placement region is a rectangle, and an object may be dropped anywhere in it as long
as the object's own footprint stays inside. Turn the object and its footprint grows
along the region's axes, so the positions that still fit shrink. Position and
orientation therefore constrain each other, and the set of valid placements is not a
product of independent ranges.

A single :class:`~random_events.product_algebra.SimpleEvent` is exactly such a product,
so it cannot hold that set. It can only bracket it: a sound event stays inside the valid
set and gives up placements that were actually fine, while a complete event covers the
valid set and admits placements that have to be rejected. Splitting the orientation
range into several simple sets narrows the bracket, and this module measures how many
simple sets that takes.

The measure is the acceptance rate a robot sampling from the event would see, because
that is what the coupling costs in practice: every sample outside the valid set is work
thrown away.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

import plotly.graph_objects as go

from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    MeanAndStandardDeviation,
    TypstRenderer,
)
from random_events.interval import closed
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Continuous

POSITION_X = Continuous("position_x")
"""
Position of the object along the region's first axis.
"""

POSITION_Y = Continuous("position_y")
"""
Position of the object along the region's second axis.
"""

ORIENTATION = Continuous("orientation")
"""
Yaw of the object about the region's normal.
"""

ORIENTATION_INTEGRATION_STEPS = 4000
"""
Slices used when integrating the valid set over the orientation range.

The integrand is smooth and bounded, so this many slices puts the quadrature error far
below the differences between the approximations being compared.
"""


@dataclass(frozen=True)
class Footprint:
    """
    The axis aligned extent of an object's base, as seen from above.
    """

    half_width: float
    """
    Half extent along the object's own first axis.
    """

    half_depth: float
    """
    Half extent along the object's own second axis.
    """

    def extents_at(self, yaw: float) -> tuple[float, float]:
        """
        :param yaw: Rotation of the object about the region's normal.
        :return: Half extents of the footprint's axis aligned bounding box at that yaw,
            which is what has to fit inside the region.
        """
        cosine, sine = abs(math.cos(yaw)), abs(math.sin(yaw))
        return (
            self.half_width * cosine + self.half_depth * sine,
            self.half_width * sine + self.half_depth * cosine,
        )

    @property
    def isotropic_buffer(self) -> float:
        """
        :return: The orientation independent margin a sampler keeps from the region's
            edge when it narrows its proposal without consulting the yaw.
        """
        return min(self.half_width, self.half_depth)

    @property
    def critical_yaws(self) -> list[float]:
        """
        :return: Every yaw at which a bounding extent of the footprint reaches an
            extremum, covering whole turns in both directions.

        Each extent is a sum of a sine and a cosine in absolute value, so it turns at
        the quarter turns, where the absolute values fold, and where the footprint's
        diagonal lines up with an axis.
        """
        diagonals = [
            math.atan2(self.half_depth, self.half_width),
            math.atan2(self.half_width, self.half_depth),
        ]
        offsets = [0.0] + diagonals + [-diagonal for diagonal in diagonals]
        return [
            offset + quarter_turn * math.pi / 2
            for offset in offsets
            for quarter_turn in range(-4, 5)
        ]


@dataclass(frozen=True)
class OrientationRange:
    """
    The yaws an object may be placed at.
    """

    lower: float
    """
    Smallest admissible yaw.
    """

    upper: float
    """
    Largest admissible yaw.
    """

    @property
    def span(self) -> float:
        """:return: Width of the range, zero when the yaw is pinned."""
        return self.upper - self.lower

    def slices(self, count: int) -> list[OrientationRange]:
        """
        :param count: How many equal sub ranges to cut the range into.
        :return: The sub ranges, one per simple set an approximation may spend.
        """
        width = self.span / count
        return [
            OrientationRange(self.lower + index * width, self.lower + (index + 1) * width)
            for index in range(count)
        ]

    def sample_points(self, count: int) -> list[float]:
        """
        :param count: How many points to take.
        :return: Midpoints of equal sub ranges, for integrating over the range.
        """
        width = self.span / count
        return [self.lower + (index + 0.5) * width for index in range(count)]


@dataclass(frozen=True)
class PlacementProblem:
    """
    A rectangular region an object of a given footprint may be placed in, at any yaw
    within a range.

    The region is taken to be axis aligned, which a survey of the RoboCasa kitchen
    layouts showed to hold for every placement frame in the suite.
    """

    region_half_width: float
    """
    Half extent of the region along its first axis.
    """

    region_half_depth: float
    """
    Half extent of the region along its second axis.
    """

    footprint: Footprint
    """
    Footprint of the object being placed.
    """

    orientation_range: OrientationRange
    """
    Yaws the object may be placed at.
    """

    def position_extents_at(self, yaw: float) -> tuple[float, float]:
        """
        :param yaw: Rotation of the object.
        :return: Half extents of the positions that keep the object inside the region,
            clamped at zero when the object no longer fits at all.
        """
        footprint_width, footprint_depth = self.footprint.extents_at(yaw)
        return (
            max(0.0, self.region_half_width - footprint_width),
            max(0.0, self.region_half_depth - footprint_depth),
        )

    def position_area_at(self, yaw: float) -> float:
        """
        :param yaw: Rotation of the object.
        :return: Area of the positions that keep the object inside the region.
        """
        half_width, half_depth = self.position_extents_at(yaw)
        return 4.0 * half_width * half_depth

    @property
    def valid_volume(self) -> float:
        """
        :return: Volume of the valid set in position and orientation together, obtained
            by integrating the admissible area over the orientation range.
        """
        if self.orientation_range.span == 0.0:
            return self.position_area_at(self.orientation_range.lower)
        points = self.orientation_range.sample_points(ORIENTATION_INTEGRATION_STEPS)
        width = self.orientation_range.span / ORIENTATION_INTEGRATION_STEPS
        return sum(self.position_area_at(yaw) for yaw in points) * width

    @property
    def sampler_proposal_volume(self) -> float:
        """
        :return: Volume of the proposal a sampler makes when it narrows the region by an
            orientation independent margin, which is what RoboCasa does before rejecting
            the placements that turn out not to fit.
        """
        buffer = self.footprint.isotropic_buffer
        half_width = max(0.0, self.region_half_width - buffer)
        half_depth = max(0.0, self.region_half_depth - buffer)
        area = 4.0 * half_width * half_depth
        if self.orientation_range.span == 0.0:
            return area
        return area * self.orientation_range.span

    @property
    def sampler_acceptance_rate(self) -> float:
        """
        :return: Share of a hand written sampler's proposals that really fit, giving the
            baseline a product algebra event has to beat.
        """
        proposal = self.sampler_proposal_volume
        if proposal <= 0.0:
            return 0.0
        return self.valid_volume / proposal

    def extreme_extents_over(
        self, orientation_slice: OrientationRange, largest: bool
    ) -> tuple[float, float]:
        """
        The extents are extremal either at the slice's edges or at one of the
        footprint's critical yaws, so evaluating exactly those points is exact rather
        than merely dense.

        :param orientation_slice: The yaws the extents are taken over.
        :param largest: Whether to take the largest admissible position extents rather
            than the smallest.
        :return: The extreme position half extents over the slice.
        """
        if orientation_slice.span == 0.0:
            return self.position_extents_at(orientation_slice.lower)
        interior = [
            yaw
            for yaw in self.footprint.critical_yaws
            if orientation_slice.lower < yaw < orientation_slice.upper
        ]
        candidates = [
            self.position_extents_at(yaw)
            for yaw in [orientation_slice.lower, orientation_slice.upper] + interior
        ]
        choose = max if largest else min
        return (
            choose(extent[0] for extent in candidates),
            choose(extent[1] for extent in candidates),
        )


class ApproximationSide(enum.Enum):
    """
    Which side of the valid set an event built from whole simple sets falls on.

    Every member's value describes what a robot sampling from such an event experiences.
    """

    SOUND = (
        "Stays inside the valid set, so every sample is a placement that fits while "
        "some placements that would have fit are never offered."
    )
    COMPLETE = (
        "Covers the valid set, so no placement that fits is ruled out while some "
        "samples have to be rejected."
    )

    @property
    def description(self) -> str:
        """:return: What this side costs a robot sampling from it."""
        return self.value


@dataclass
class SlicedApproximation:
    """
    Approximates the valid set by one simple event per slice of the orientation range.

    Cutting only along orientation is what a robot can do without knowing the
    footprint's shape, and it is the cut that addresses the coupling directly: within a
    narrow enough slice the admissible positions barely move.
    """

    problem: PlacementProblem
    """
    The placement being approximated.
    """

    number_of_simple_sets: int
    """
    How many orientation slices, and therefore simple events, may be spent.
    """

    side: ApproximationSide
    """
    Which side of the valid set the approximation falls on.
    """

    def __post_init__(self):
        assert (
            self.number_of_simple_sets > 0
        ), "A valid set cannot be approximated by zero simple sets."

    @property
    def slices(self) -> list[OrientationRange]:
        """:return: The orientation slices the approximation is built from."""
        return self.problem.orientation_range.slices(self.number_of_simple_sets)

    def extents_for(self, orientation_slice: OrientationRange) -> tuple[float, float]:
        """
        :param orientation_slice: The slice to size a simple event for.
        :return: Position half extents this slice's simple event uses.
        """
        return self.problem.extreme_extents_over(
            orientation_slice, largest=self.side is ApproximationSide.COMPLETE
        )

    @property
    def volume(self) -> float:
        """:return: Volume of the approximation in position and orientation
        together."""
        if self.problem.orientation_range.span == 0.0:
            half_width, half_depth = self.extents_for(self.problem.orientation_range)
            return 4.0 * half_width * half_depth
        return sum(
            4.0
            * self.extents_for(orientation_slice)[0]
            * self.extents_for(orientation_slice)[1]
            * orientation_slice.span
            for orientation_slice in self.slices
        )

    @property
    def event(self) -> Event:
        """:return: The approximation as a product algebra event."""
        simple_events = []
        for orientation_slice in self.slices:
            half_width, half_depth = self.extents_for(orientation_slice)
            if half_width <= 0.0 or half_depth <= 0.0:
                continue
            simple_events.append(
                SimpleEvent.from_data(
                    {
                        POSITION_X: closed(-half_width, half_width),
                        POSITION_Y: closed(-half_depth, half_depth),
                        ORIENTATION: closed(
                            orientation_slice.lower, orientation_slice.upper
                        ),
                    }
                )
            )
        return Event.from_simple_sets(*simple_events)


@dataclass
class CouplingCost(ExperimentResult):
    """
    What a fixed number of simple sets achieves on one placement problem.
    """

    number_of_simple_sets: int
    """
    Simple sets the approximations were allowed to use.
    """

    acceptance_rate: float
    """
    Share of samples drawn from the complete approximation that really fit, so one minus
    the share a robot has to reject.
    """

    offered_share: float
    """
    Share of the valid set the sound approximation still offers, so one minus the share
    of workable placements silently given up.
    """

    @classmethod
    def measure(
        cls, problem: PlacementProblem, number_of_simple_sets: int
    ) -> CouplingCost:
        """
        :param problem: The placement to measure.
        :param number_of_simple_sets: Simple sets each approximation may use.
        :return: What both approximations achieve at that budget.
        """
        sound = SlicedApproximation(
            problem, number_of_simple_sets, ApproximationSide.SOUND
        )
        complete = SlicedApproximation(
            problem, number_of_simple_sets, ApproximationSide.COMPLETE
        )
        valid_volume = problem.valid_volume
        return cls(
            number_of_simple_sets=number_of_simple_sets,
            acceptance_rate=round(valid_volume / complete.volume, 4)
            if complete.volume > 0.0
            else 0.0,
            offered_share=round(sound.volume / valid_volume, 4)
            if valid_volume > 0.0
            else 0.0,
        )


@dataclass
class TightnessCost(ExperimentResult):
    """
    What the coupling costs at one ratio of object footprint to region size.
    """

    tightness: float
    """
    Largest footprint half extent as a share of the smallest region half extent, so
    higher means the object more nearly fills the region.
    """

    acceptance_with_one_simple_set: float
    """
    Share of samples that fit when the whole placement is held in a single simple event.
    """

    simple_sets_for_ninety_five_percent: int
    """
    Simple sets needed before the complete approximation accepts nineteen samples in
    twenty.
    """

    simple_sets_for_ninety_nine_percent: int
    """
    Simple sets needed before the complete approximation accepts ninety nine samples in
    a hundred.
    """


@dataclass
class OrientationCouplingExperiment:
    """
    Measures the cost of the coupling as an object grows relative to its region.
    """

    region_half_extents: tuple[float, float] = (0.20, 0.15)
    """
    Half extents of the region, in metres.
    """

    orientation_range: OrientationRange = OrientationRange(-math.pi / 4, math.pi / 4)
    """
    Yaws the object may be placed at, matching the range RoboCasa uses when a task does
    not state one.
    """

    footprint_aspect_ratio: float = 1.5
    """
    Ratio of the footprint's long side to its short side.

    A square footprint is the degenerate case whose bounding box grows most slowly with
    yaw, so a mildly elongated object is the more representative choice.
    """

    tightness_values: tuple[float, ...] = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)
    """
    Footprint sizes to measure, as a share of the smallest region half extent.
    """

    simple_set_budgets: tuple[int, ...] = (1, 2, 4, 8, 16, 32, 64)
    """
    Simple set counts to measure at.
    """

    search_limit: int = 2048
    """
    Largest number of simple sets considered when searching for a target acceptance
    rate.
    """

    def problem_at(self, tightness: float) -> PlacementProblem:
        """
        :param tightness: Footprint size as a share of the smallest region half extent.
        :return: The placement problem at that footprint size.
        """
        half_width, half_depth = self.region_half_extents
        long_side = tightness * min(half_width, half_depth)
        return PlacementProblem(
            region_half_width=half_width,
            region_half_depth=half_depth,
            footprint=Footprint(long_side, long_side / self.footprint_aspect_ratio),
            orientation_range=self.orientation_range,
        )

    def cost_table(self, tightness: float) -> ExperimentsTable:
        """
        :param tightness: Footprint size as a share of the smallest region half extent.
        :return: What each simple set budget achieves at that footprint size.
        """
        problem = self.problem_at(tightness)
        return ExperimentsTable(
            [
                CouplingCost.measure(problem, budget)
                for budget in self.simple_set_budgets
            ]
        )

    def budget_for(self, tightness: float, target_acceptance: float) -> int:
        """
        :param tightness: Footprint size as a share of the smallest region half extent.
        :param target_acceptance: Acceptance rate the complete approximation must reach.
        :return: Smallest sufficient number of simple sets, or the search limit when the
            target is never reached.
        """
        problem = self.problem_at(tightness)
        number_of_simple_sets = 1
        while number_of_simple_sets <= self.search_limit:
            cost = CouplingCost.measure(problem, number_of_simple_sets)
            if cost.acceptance_rate >= target_acceptance:
                return number_of_simple_sets
            number_of_simple_sets *= 2
        return self.search_limit

    def tightness_table(self) -> ExperimentsTable:
        """:return: The cost of the coupling across every measured footprint size."""
        return ExperimentsTable(
            [
                TightnessCost(
                    tightness=tightness,
                    acceptance_with_one_simple_set=CouplingCost.measure(
                        self.problem_at(tightness), 1
                    ).acceptance_rate,
                    simple_sets_for_ninety_five_percent=self.budget_for(
                        tightness, 0.95
                    ),
                    simple_sets_for_ninety_nine_percent=self.budget_for(
                        tightness, 0.99
                    ),
                )
                for tightness in self.tightness_values
            ]
        )


@dataclass
class SurveyedPlacement:
    """
    One placement as RoboCasa actually resolved it, reduced to the quantities that drive
    the coupling.
    """

    task: str
    """
    Task the placement belongs to.
    """

    region_half_width: float
    """
    Half extent of the resolved region along its first axis.
    """

    region_half_depth: float
    """
    Half extent of the resolved region along its second axis.
    """

    footprint: Footprint
    """
    Footprint of the object that was placed.
    """

    orientation_range: OrientationRange
    """
    Yaws the object could be placed at.
    """

    fit_is_enforced: bool
    """
    Whether the object's footprint was required to stay inside the region, which is what
    ties its position to its orientation.
    """

    @classmethod
    def from_record(cls, record: dict[str, object]) -> SurveyedPlacement:
        """
        :param record: One entry of a placement survey.
        :return: The placement it describes.
        """
        return cls(
            task=str(record["task"]),
            region_half_width=float(record["region_width"]) / 2,
            region_half_depth=float(record["region_depth"]) / 2,
            footprint=Footprint(
                float(record["footprint_width"]) / 2,
                float(record["footprint_depth"]) / 2,
            ),
            orientation_range=OrientationRange(
                float(record["yaw_lower"]), float(record["yaw_upper"])
            ),
            fit_is_enforced=bool(record["fit_is_enforced"]),
        )

    @property
    def problem(self) -> PlacementProblem:
        """:return: The placement problem this survey entry poses."""
        return PlacementProblem(
            region_half_width=self.region_half_width,
            region_half_depth=self.region_half_depth,
            footprint=self.footprint,
            orientation_range=self.orientation_range,
        )

    @property
    def tightness(self) -> float:
        """:return: Largest footprint half extent as a share of the smallest region half
        extent."""
        smallest_region_extent = min(self.region_half_width, self.region_half_depth)
        largest_footprint_extent = max(
            self.footprint.half_width, self.footprint.half_depth
        )
        if smallest_region_extent <= 0.0:
            return math.inf
        return largest_footprint_extent / smallest_region_extent

    @property
    def is_coupled(self) -> bool:
        """
        :return: Whether the placement ties position to orientation at all, which needs
            both a yaw that can vary and a footprint that has to stay inside the region.
        """
        return self.fit_is_enforced and self.orientation_range.span > 0.0


@dataclass
class SurveySummary(ExperimentResult):
    """
    What the coupling costs across the placements a survey observed.
    """

    placements: int
    """
    Placements the summary covers.
    """

    median_tightness: float
    """
    Median share of the region the placed objects take up.
    """

    median_acceptance_with_one_simple_set: float
    """
    Median share of samples that fit when a placement is held in a single simple event.
    """

    placements_below_ninety_five_percent: int
    """
    Placements a single simple event cannot hold at an acceptance rate of nineteen in
    twenty.
    """

    uncoupled_placements: int
    """
    Observed placements that pin the yaw or do not enforce the fit, which a single
    simple event holds exactly and which the summary therefore excludes.
    """

    infeasible_placements: int
    """
    Coupled placements whose object does not fit the region at any yaw under the bounding
    box test used here, which the summary also excludes.
    """


@dataclass
class PlacementSurvey:
    """
    The placements observed while resetting RoboCasa tasks across its layouts.
    """

    placements: list[SurveyedPlacement]
    """
    The observed placements the coupling actually applies to.
    """

    uncoupled_placements: int = 0
    """
    Observed placements that pin the yaw or do not enforce the fit.
    """

    infeasible_placements: int = 0
    """
    Coupled placements whose object does not fit at any yaw under the bounding box test.
    """

    @classmethod
    def from_records(cls, records: list[dict[str, object]]) -> PlacementSurvey:
        """
        Split the observed placements into the ones the coupling applies to and the ones
        it does not, so that neither group is silently folded into the other.

        :param records: Entries of a placement survey.
        :return: The survey they describe.
        """
        observed = [SurveyedPlacement.from_record(record) for record in records]
        coupled = [placement for placement in observed if placement.is_coupled]
        feasible = [
            placement for placement in coupled if placement.problem.valid_volume > 0.0
        ]
        return cls(
            placements=feasible,
            uncoupled_placements=len(observed) - len(coupled),
            infeasible_placements=len(coupled) - len(feasible),
        )

    def acceptance_rates(self, number_of_simple_sets: int) -> list[float]:
        """
        :param number_of_simple_sets: Simple sets each placement may use.
        :return: The acceptance rate reached on every observed placement.
        """
        return [
            CouplingCost.measure(placement.problem, number_of_simple_sets).acceptance_rate
            for placement in self.placements
        ]

    def tightness_statistics(self) -> MeanAndStandardDeviation:
        """:return: Mean and spread of how much of its region an object takes up."""
        return MeanAndStandardDeviation.from_measurements(
            [placement.tightness for placement in self.placements]
        )

    def summarize(self) -> SurveySummary:
        """:return: What a single simple event achieves across the survey."""
        rates = sorted(self.acceptance_rates(1))
        tightnesses = sorted(placement.tightness for placement in self.placements)
        return SurveySummary(
            placements=len(self.placements),
            median_tightness=round(self.median(tightnesses), 3),
            median_acceptance_with_one_simple_set=round(self.median(rates), 4),
            placements_below_ninety_five_percent=sum(
                1 for rate in rates if rate < 0.95
            ),
            uncoupled_placements=self.uncoupled_placements,
            infeasible_placements=self.infeasible_placements,
        )

    @staticmethod
    def median(values: list[float]) -> float:
        """
        :param values: Values in ascending order.
        :return: Their median, or zero when there are none.
        """
        if not values:
            return 0.0
        middle = len(values) // 2
        if len(values) % 2 == 1:
            return values[middle]
        return (values[middle - 1] + values[middle]) / 2


SIDE_COLORS = {
    ApproximationSide.SOUND: "#2a78d6",
    ApproximationSide.COMPLETE: "#eb6834",
}
"""
Fixed categorical color per approximation side.
"""


def plot_coupling_cost(experiment: OrientationCouplingExperiment) -> go.Figure:
    """
    Show how the acceptance rate improves as the simple set budget grows, one trace per
    footprint size.

    :param experiment: The experiment whose measurements are plotted.
    :return: A figure comparing every measured footprint size.
    """
    figure = go.Figure()
    for tightness in experiment.tightness_values:
        problem = experiment.problem_at(tightness)
        costs = [
            CouplingCost.measure(problem, budget)
            for budget in experiment.simple_set_budgets
        ]
        figure.add_trace(
            go.Scatter(
                x=list(experiment.simple_set_budgets),
                y=[cost.acceptance_rate for cost in costs],
                mode="lines+markers",
                name=f"footprint {tightness:.0%} of region",
                line=dict(color=SIDE_COLORS[ApproximationSide.COMPLETE], width=2),
                opacity=0.3 + 0.7 * tightness / max(experiment.tightness_values),
                hovertemplate=(
                    f"Footprint: {tightness:.0%} of region<br>"
                    "Simple sets: %{x}<br>"
                    "Acceptance rate: %{y:.3f}<extra></extra>"
                ),
            )
        )

    figure.update_layout(
        title="Cost of the Orientation Coupling",
        xaxis_title="Simple Sets in the Approximation",
        yaxis_title="Share of Samples That Fit",
        xaxis_type="log",
        plot_bgcolor="#fcfcfb",
        paper_bgcolor="#fcfcfb",
        font=dict(color="#0b0b0b"),
    )
    figure.update_xaxes(gridcolor="#e1e0d9", zerolinecolor="#c3c2b7")
    figure.update_yaxes(gridcolor="#e1e0d9", zerolinecolor="#c3c2b7", range=[0, 1.02])
    return figure


def main():
    experiment = OrientationCouplingExperiment()
    print(
        TypstRenderer(experiment.tightness_table()).render_figure(
            "Cost of the orientation coupling as an object grows relative to its "
            "placement region. Acceptance is the share of samples drawn from a complete "
            "approximation that really fit, so a single simple set holding the whole "
            "placement wastes the remainder."
        )
    )
    print()
    print(
        TypstRenderer(experiment.cost_table(0.5)).render_figure(
            "Both approximations of a placement whose object spans half its region, as "
            "the simple set budget grows."
        )
    )


if __name__ == "__main__":
    main()

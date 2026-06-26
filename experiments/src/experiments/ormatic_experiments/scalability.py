"""
Scalability experiments for the ORMatic framework and the episodic query system.

Two orthogonal scalability dimensions are covered:

1. **Schema generation scalability** (:func:`run_scalability_experiment`): measures
   how fast ORMatic generates ORM code as the number of mapped classes grows.

2. **Query execution scalability** (:func:`run_query_scalability_experiment`): measures
   how fast SQL queries execute as the number of stored execution episodes grows.
   This directly validates Requirement 5 (Scalable Data Storage), which states that
   query performance must not degrade as the episodic database grows.

Four representative SQL queries are selected from :mod:`experiments.querying`,
covering the main SQL complexity classes:

- **Filter**: return all succeeded action nodes.
- **Ordered traversal**: return all plan nodes ordered by start time.
- **Aggregation**: compute the earliest start and latest end time.
- **Group-by**: count nodes by status.
"""

import logging
import random
import time
import tempfile
from contextlib import contextmanager
from dataclasses import is_dataclass, dataclass
from typing import Any, Type, List, Set, Tuple

import plotly.graph_objects as go
import tqdm
from sqlalchemy.orm import Session, sessionmaker

import coraplex.orm.ormatic_interface  # type: ignore
import coraplex.plans.plan_node
import semantic_digital_twin  # type: ignore
from coraplex.orm.ormatic_interface import Base  # type: ignore
from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    MeanAndStandardDeviation,
    MedianAndIQR,
    TypstRenderer,
)
from experiments.querying import build_plan, build_queries
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.data_access_objects.alternative_mappings import AlternativeMapping
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.ormatic.eql_interface import eql_to_sql
from krrood.ormatic.helper import get_classes_of_ormatic_interface
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.type_dict import TypeDict
from krrood.ormatic.utils import create_engine, drop_database
from krrood.utils import recursive_subclasses
from coraplex.robot_plans.actions.base import ActionDescription

_EPISODE_COUNTS: List[int] = [1, 10, 50, 100, 500, 1000]
_QUERY_REPETITIONS: int = 10

# Indices into the list returned by build_queries() for the four representative
# queries. These are stable as long as build_queries() preserves its order.
_FILTER_INDEX: int = 0
_TRAVERSAL_INDEX: int = 3
_AGGREGATION_INDEX: int = 2
_GROUPBY_INDEX: int = 9


def build_cram_class_sets() -> Tuple[Set[Type], List[Type], dict]:
    """
    Collect all mappable classes, alternative mappings, and type mappings from
    the coraplex ORM interface.

    Filters out non-dataclasses and AlternativeMapping subclasses from the raw
    interface, then augments with the original classes of every registered
    AlternativeMapping so the full set is consistent.

    :return: Tuple of (classes, alternative_mappings, type_mappings) ready to
             pass to :func:`run_scalability_experiment`.
    """
    classes, alternative_mappings, type_mappings = get_classes_of_ormatic_interface(
        coraplex.orm.ormatic_interface
    )
    classes = set(classes)

    alternative_mappings += [
        alternative_mapping
        for alternative_mapping in recursive_subclasses(AlternativeMapping)
    ]
    alternative_mappings = list(set(alternative_mappings))
    classes = {
        c for c in classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
    }
    classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}
    alternative_mappings = [
        am
        for am in recursive_subclasses(AlternativeMapping)
        if am.original_class() in classes
    ]
    return classes, alternative_mappings, type_mappings


@dataclass
class ORMaticScalabilityExperimentResult(ExperimentResult):
    """
    Raw measurements from a single ORMatic generation run.

    All durations are in seconds, rounded to two decimal places.
    Structural counts reflect the class diagram that was actually built for
    the given filtered class set.
    """

    total_duration: float
    """Wall-clock time from ClassDiagram creation to file write completion."""
    class_diagram_creation_duration: float
    """Time spent constructing the ClassDiagram."""
    ormatic_reasoning_duration: float
    """Time spent in ORMatic.make_all_tables()."""
    writing_to_file_duration: float
    """Time spent serialising the generated SQLAlchemy code to a temp file."""
    number_of_classes: int
    """Number of classes in the filtered input set."""
    number_of_associations: int
    """Number of association edges in the resulting class diagram."""
    number_of_inheritances: int
    """Number of inheritance edges in the resulting class diagram."""


@dataclass
class ORMaticScalabilityAggregateResult(ExperimentResult):
    """
    Aggregated statistics over multiple ORMatic generation runs at a fixed drop probability.

    Every numeric field is a :class:`MeanAndStandardDeviation` computed across
    all iterations of :func:`run_scalability_experiment`.  Structural counts
    (classes, associations, inheritances) vary between iterations because the
    class subset is resampled each time.
    """

    class_drop_probability: float
    """Fraction of classes randomly excluded from each iteration's input set."""
    number_of_classes: MeanAndStandardDeviation
    """Statistics over the size of the filtered class set across iterations."""
    number_of_associations: MeanAndStandardDeviation
    """Statistics over association edge count across iterations."""
    number_of_inheritances: MeanAndStandardDeviation
    """Statistics over inheritance edge count across iterations."""
    total_duration: MeanAndStandardDeviation
    """Statistics over total generation time (s) across iterations."""
    class_diagram_creation_duration: MeanAndStandardDeviation
    """Statistics over ClassDiagram construction time (s) across iterations."""
    ormatic_reasoning_duration: MeanAndStandardDeviation
    """Statistics over ORMatic reasoning time (s) across iterations."""
    writing_to_file_duration: MeanAndStandardDeviation
    """Statistics over file serialisation time (s) across iterations."""


@contextmanager
def _silence_ormatic_logger():
    logger = logging.getLogger("krrood.ormatic")
    original_level = logger.level
    logger.setLevel(logging.CRITICAL)
    try:
        yield
    finally:
        logger.setLevel(original_level)


def ormatic_scalability_experiment(
    filtered_classes: Set[Type],
    alternative_mappings: List[Type],
    type_mappings: dict,
) -> ORMaticScalabilityExperimentResult:
    """
    Run a single ORMatic generation pass over a pre-determined class set and
    return timing and structural measurements.

    ORMatic log output is suppressed for the duration of the run to keep
    benchmark output readable.

    :param filtered_classes: The exact set of classes to map in this run.
    :param alternative_mappings: AlternativeMapping subclasses to register with ORMatic.
    :param type_mappings: Custom type-to-column mappings forwarded to :class:`TypeDict`.
    :return: Timing breakdown and class-diagram statistics for this single run.
    """
    with _silence_ormatic_logger():
        return _ormatic_scalability_experiment(
            filtered_classes, alternative_mappings, type_mappings
        )


def _ormatic_scalability_experiment(
    filtered_classes: Set[Type],
    alternative_mappings: List[Type],
    type_mappings: dict,
) -> ORMaticScalabilityExperimentResult:
    """
    Inner implementation of a single ORMatic generation pass without logger suppression.

    Measures three sequential phases: ClassDiagram construction, ORMatic table reasoning
    (``make_all_tables``), and writing the generated SQLAlchemy file to a temp file.

    :param filtered_classes: The exact set of classes to map in this run.
    :param alternative_mappings: AlternativeMapping subclasses to register with ORMatic.
    :param type_mappings: Custom type-to-column mappings forwarded to :class:`TypeDict`.
    :return: Timing breakdown and class-diagram statistics for this single run.
    """
    begin = time.perf_counter()

    class_diagram = ClassDiagram(
        list(sorted(filtered_classes, key=lambda c: c.__name__, reverse=True))
    )

    class_diagram_creation_time = time.perf_counter()

    ormatic = ORMatic(
        class_diagram,
        type_mappings=TypeDict(type_mappings),
        alternative_mappings=alternative_mappings,
    )
    ormatic.make_all_tables()

    ormatic_reasoning_time = time.perf_counter()

    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".py") as f:
        ormatic.to_sqlalchemy_file(f)

    writing_to_file_time = time.perf_counter()

    return ORMaticScalabilityExperimentResult(
        total_duration=round(writing_to_file_time - begin, 2),
        class_diagram_creation_duration=round(class_diagram_creation_time - begin, 2),
        ormatic_reasoning_duration=round(
            ormatic_reasoning_time - class_diagram_creation_time, 2
        ),
        writing_to_file_duration=round(
            writing_to_file_time - ormatic_reasoning_time, 2
        ),
        number_of_classes=len(filtered_classes),
        number_of_associations=len(class_diagram.associations),
        number_of_inheritances=len(class_diagram.inheritance_relations),
    )


def run_scalability_experiment(
    classes: List[Type],
    alternative_mappings: List[Type],
    type_mappings: dict,
    class_drop_probability: float = 0.3,
    iterations: int = 10,
    required_classes: List[Type] | None = None,
) -> ORMaticScalabilityAggregateResult:
    """
    Repeatedly sample a random subset of classes and run the ORMatic generation
    pipeline, then aggregate timing and structural measurements across all runs.

    Each iteration independently resamples the class subset so that the reported
    standard deviations reflect variability from both class-set composition and
    runtime noise.  AlternativeMapping original classes and any ``required_classes``
    are always present in every iteration's input, regardless of the drop probability.

    :param classes: Full pool of candidate classes to sample from.
    :param alternative_mappings: AlternativeMapping subclasses to register with ORMatic.
    :param type_mappings: Custom type-to-column mappings forwarded to :class:`TypeDict`.
    :param class_drop_probability: Per-class probability of exclusion in each iteration.
    :param iterations: Number of independent generation runs to aggregate.
    :param required_classes: Classes pinned into every iteration's input set.
    :return: Aggregated mean and standard deviation for all measurements.
    """
    pinned = set(required_classes) if required_classes else set()
    results = []
    for _ in range(iterations):
        filtered_classes = {
            c for c in classes if random.uniform(0, 1) > class_drop_probability
        }
        filtered_classes |= {
            alternative_mapping.original_class()
            for alternative_mapping in alternative_mappings
        }
        filtered_classes |= pinned
        results.append(
            ormatic_scalability_experiment(
                filtered_classes, alternative_mappings, type_mappings
            )
        )

    return ORMaticScalabilityAggregateResult(
        class_drop_probability=class_drop_probability,
        number_of_classes=MeanAndStandardDeviation.from_measurements(
            [r.number_of_classes for r in results]
        ),
        number_of_associations=MeanAndStandardDeviation.from_measurements(
            [r.number_of_associations for r in results]
        ),
        number_of_inheritances=MeanAndStandardDeviation.from_measurements(
            [r.number_of_inheritances for r in results]
        ),
        total_duration=MeanAndStandardDeviation.from_measurements(
            [r.total_duration for r in results]
        ),
        class_diagram_creation_duration=MeanAndStandardDeviation.from_measurements(
            [r.class_diagram_creation_duration for r in results]
        ),
        ormatic_reasoning_duration=MeanAndStandardDeviation.from_measurements(
            [r.ormatic_reasoning_duration for r in results]
        ),
        writing_to_file_duration=MeanAndStandardDeviation.from_measurements(
            [r.writing_to_file_duration for r in results]
        ),
    )


def plot_scalability(table: ExperimentsTable) -> go.Figure:
    """
    Produce a band plot of number of classes (x) vs mean total runtime (y).

    The shaded band covers mean ± 1 standard deviation of the total duration.
    The x-axis uses the mean number of classes from each
    :class:`ORMaticScalabilityAggregateResult` row in ``table``.

    :param table: An :class:`ExperimentsTable` whose rows are
                  :class:`ORMaticScalabilityAggregateResult` instances, typically
                  produced by calling :func:`run_scalability_experiment` at
                  several different drop probabilities.
    :return: A Plotly figure ready for display or export.
    """
    rows: List[ORMaticScalabilityAggregateResult] = table.experiments

    x = [r.number_of_classes.mean for r in rows]
    y = [r.total_duration.mean for r in rows]
    y_upper = [
        r.total_duration.mean + r.total_duration.standard_deviation for r in rows
    ]
    y_lower = [
        r.total_duration.mean - r.total_duration.standard_deviation for r in rows
    ]

    fig = go.Figure(
        [
            go.Scatter(
                x=x + x[::-1],
                y=y_upper + y_lower[::-1],
                fill="toself",
                fillcolor="rgba(0, 100, 250, 0.2)",
                line=dict(color="rgba(0, 0, 0, 0)"),
                showlegend=True,
                name="±1 std",
            ),
            go.Scatter(
                x=x,
                y=y,
                mode="lines+markers",
                line=dict(color="rgb(0, 100, 250)"),
                name="mean",
            ),
        ]
    )
    fig.update_layout(
        xaxis_title="Number of Classes",
        yaxis_title="Total Duration (s)",
        title="ORMatic Scalability: Classes vs Runtime",
    )
    return fig


@dataclass
class QueryScalabilityResult(ExperimentResult):
    """
    One row in the query scalability table.

    Each row reports median and IQR of SQL execution time for four
    representative query types at a specific episode count.
    """

    episode_count: int
    """Number of stored execution episodes in the database for this row."""

    filter_duration_ms: MedianAndIQR
    """
    SQL execution time for a simple status-equality filter query across all
    stored episodes (``What did you just do?``).
    """

    traversal_duration_ms: MedianAndIQR
    """
    SQL execution time for an ordered traversal query that returns all plan
    nodes sorted by start time (``How long did each step take?``).
    """

    aggregation_duration_ms: MedianAndIQR
    """
    SQL execution time for a min/max aggregation query that returns a single
    row (``How long did the whole task take?``).
    """

    groupby_duration_ms: MedianAndIQR
    """
    SQL execution time for a grouped-count query
    (``Were all subtasks successful, or did some fail?``).
    """


def _populate_database(session: Session, plan: Any, episode_count: int) -> None:
    """
    Write *episode_count* copies of *plan* to the database behind *session*.

    :param session: An open SQLAlchemy session connected to an empty database.
    :param plan: The fully executed plan to serialise.
    :param episode_count: Number of copies to store.
    """
    for _ in range(episode_count):
        session.add(to_dao(plan))
    session.commit()


def _time_sql_query(translator: Any, repetitions: int) -> MedianAndIQR:
    """
    Execute a pre-translated SQL query *repetitions* times and return timing statistics.

    :param translator: A translator object whose ``.evaluate()`` executes the
        SQL query against the connected database.
    :param repetitions: Number of timed executions.
    :return: Median and IQR of execution times in milliseconds.
    """
    timings: List[float] = []
    for _ in range(repetitions):
        t0 = time.perf_counter()
        translator.evaluate()
        timings.append((time.perf_counter() - t0) * 1000.0)
    return MedianAndIQR.from_measurements(timings)


def run_query_scalability_experiment(
    episode_counts: List[int] = _EPISODE_COUNTS,
    repetitions: int = _QUERY_REPETITIONS,
) -> ExperimentsTable:
    """
    Measure SQL query execution time across a range of episode counts.

    For each episode count a fresh in-memory SQLite database is populated and
    four representative SQL queries are timed. The database is discarded after
    each episode count to ensure isolation between measurements. This directly
    validates Requirement 5 (Scalable Data Storage): as the episodic database
    grows, query performance must remain acceptable.

    :param episode_counts: Ordered list of episode counts to test.
    :param repetitions: Number of timed query executions per (count, query) pair.
    :return: A table with one :class:`QueryScalabilityResult` row per episode count.
    """
    plan = build_plan()
    queries = build_queries([plan])

    filter_query = queries[_FILTER_INDEX]
    traversal_query = queries[_TRAVERSAL_INDEX]
    aggregation_query = queries[_AGGREGATION_INDEX]
    groupby_query = queries[_GROUPBY_INDEX]

    rows: List[QueryScalabilityResult] = []

    for episode_count in tqdm.tqdm(episode_counts):
        engine = create_engine("sqlite:///:memory:")
        session = sessionmaker(engine)()
        Base.metadata.create_all(bind=session.bind)

        _populate_database(session, plan, episode_count)

        filter_translator = eql_to_sql(filter_query.query, session)
        traversal_translator = eql_to_sql(traversal_query.query, session)
        aggregation_translator = eql_to_sql(aggregation_query.query, session)
        groupby_translator = eql_to_sql(groupby_query.query, session)

        rows.append(
            QueryScalabilityResult(
                episode_count=episode_count,
                filter_duration_ms=_time_sql_query(filter_translator, repetitions),
                traversal_duration_ms=_time_sql_query(traversal_translator, repetitions),
                aggregation_duration_ms=_time_sql_query(aggregation_translator, repetitions),
                groupby_duration_ms=_time_sql_query(groupby_translator, repetitions),
            )
        )

        drop_database(session.bind)
        session.close()
        engine.dispose()

    return ExperimentsTable(
        rows,
        description=(
            f"SQL query execution time as a function of the number of stored episodes, "
            f"validating Requirement 5 (Scalable Data Storage). "
            f"Each row populates a fresh in-memory SQLite database with the given episode "
            f"count and times four representative SQL queries over {repetitions} repetitions; "
            f"duration columns report median [Q1, Q3] in milliseconds. "
            f"The four query types cover the main SQL complexity classes: "
            f"status-equality filter, ordered traversal, min/max aggregation, and grouped count."
        ),
    )


def main():
    classes, alternative_mappings, type_mappings = build_cram_class_sets()
    required_classes = [coraplex.plans.plan_node.UnderspecifiedNode, ActionDescription]
    results = []
    for class_drop_probability in tqdm.tqdm(
        [
            0.9,
            0.8,
            0.7,
            0.6,
            0.5,
            0.4,
            0.3,
            0.2,
            0.1,
            0.0,
        ]
    ):
        results.append(
            run_scalability_experiment(
                classes,
                alternative_mappings,
                type_mappings,
                class_drop_probability,
                iterations=10,
                required_classes=required_classes,
            )
        )

    generation_table = ExperimentsTable(
        results,
        description=(
            "ORMatic code-generation performance as a function of input class-set size, "
            "measured at ten class-drop probabilities from 0.9 (smallest subset) to 0.0 "
            "(full class set). "
            "Each row aggregates 10 independent generation runs; all numeric columns report "
            "mean ± standard deviation. "
            "Structural counts (classes, associations, inheritances) vary between runs "
            "because the class subset is resampled each time. "
            "Duration columns report seconds for class-diagram construction, "
            "ORMatic table reasoning, and SQLAlchemy file serialisation."
        ),
    )
    print(TypstRenderer(generation_table).render_table())
    plot_scalability(generation_table).show()

    query_table = run_query_scalability_experiment()
    print(TypstRenderer(query_table).render_table())


if __name__ == "__main__":
    main()

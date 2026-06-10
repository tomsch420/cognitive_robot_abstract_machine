import logging
import random
import time
import tempfile
from contextlib import contextmanager
from dataclasses import is_dataclass, dataclass
from typing import Type, List, Set

import plotly.graph_objects as go
import tqdm

import pycram.orm.ormatic_interface  # type: ignore
import semantic_digital_twin  # type: ignore
from experiments.experiment_definitions import (
    ExperimentResult,
    ExperimentsTable,
    MeanAndStandardDeviation,
    TypstRenderer,
)
import pycram.plans.plan_node
from krrood.class_diagrams import ClassDiagram
from krrood.ormatic.data_access_objects.alternative_mappings import AlternativeMapping
from krrood.ormatic.helper import get_classes_of_ormatic_interface
from krrood.ormatic.ormatic import ORMatic
from krrood.ormatic.type_dict import TypeDict
from krrood.utils import recursive_subclasses


# import classes from the existing interface
classes, alternative_mappings, type_mappings = get_classes_of_ormatic_interface(
    pycram.orm.ormatic_interface
)
classes = set(classes)

alternative_mappings += [am for am in recursive_subclasses(AlternativeMapping)]
alternative_mappings = list(set(alternative_mappings))
# keep only dataclasses that are NOT AlternativeMapping subclasses
classes = {
    c for c in classes if is_dataclass(c) and not issubclass(c, AlternativeMapping)
}
classes |= {am.original_class() for am in recursive_subclasses(AlternativeMapping)}

alternative_mappings = [
    am
    for am in recursive_subclasses(AlternativeMapping)
    if am.original_class() in classes
]


@dataclass
class ORMaticScalabilityExperimentResult(ExperimentResult):
    total_duration: float
    class_diagram_creation_duration: float
    ormatic_reasoning_duration: float
    writing_to_file_duration: float
    number_of_classes: int
    number_of_associations: int
    number_of_inheritances: int


@dataclass
class ORMaticScalabilityAggregateResult(ExperimentResult):
    class_drop_probability: float
    number_of_classes: MeanAndStandardDeviation
    number_of_associations: MeanAndStandardDeviation
    number_of_inheritances: MeanAndStandardDeviation
    total_duration: MeanAndStandardDeviation
    class_diagram_creation_duration: MeanAndStandardDeviation
    ormatic_reasoning_duration: MeanAndStandardDeviation
    writing_to_file_duration: MeanAndStandardDeviation


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
) -> ORMaticScalabilityExperimentResult:
    """
    Run a single ORMatic scalability experiment over a fixed set of classes.

    :param filtered_classes: Pre-determined set of classes to map.
    :return: Timing and structural measurements for this run.
    """
    with _silence_ormatic_logger():
        return _ormatic_scalability_experiment(filtered_classes)


def _ormatic_scalability_experiment(
    filtered_classes: Set[Type],
) -> ORMaticScalabilityExperimentResult:
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
    class_drop_probability: float = 0.3,
    iterations: int = 10,
    required_classes: List[Type] | None = None,
) -> ORMaticScalabilityAggregateResult:
    """
    Fix a random subset of classes once, then run the generation process `iterations` times
    with that same subset and aggregate mean and variance of all timing measurements.

    :param classes: Full pool of classes to sample from.
    :param class_drop_probability: Probability of dropping each class when building the subset.
    :param iterations: Number of generation runs over the fixed subset.
    :param required_classes: Classes that are always included regardless of drop probability.
    :return: Aggregated timing statistics across all runs.
    """
    pinned = set(required_classes) if required_classes else set()
    results = []
    for _ in range(iterations):
        filtered_classes = {
            c for c in classes if random.uniform(0, 1) > class_drop_probability
        }
        filtered_classes |= {am.original_class() for am in alternative_mappings}
        filtered_classes |= pinned
        results.append(ormatic_scalability_experiment(filtered_classes))

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
    Band plot of number of classes (x) vs total runtime (y) for a table of
    ORMaticScalabilityAggregateResult rows. The shaded band spans ±1 standard
    deviation around the mean runtime.
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


def main():
    required_classes = [pycram.plans.plan_node.UnderspecifiedNode]
    results = []
    for class_drop_probability in tqdm.tqdm(
        list(reversed([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]))
    ):
        results.append(
            run_scalability_experiment(classes, class_drop_probability, iterations=2)
        )

    table = ExperimentsTable(results)
    print(TypstRenderer(table).render_table())
    plot_scalability(table).show()


if __name__ == "__main__":
    main()

"""
Tests for the circuit learning benchmark: its datasets, its measurements and its runner,
on datasets placed in the cache so that nothing is downloaded.
"""

from __future__ import annotations

# %% imports
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from random_events.variable import Continuous, Symbolic

from probabilistic_model.benchmark.datasets import (
    BinaryDatasetName,
    BinaryDatasetSource,
    BinarySplit,
    MixedDatasetName,
    MixedDatasetSource,
)
from probabilistic_model.benchmark.measurements import (
    Capability,
    Measurement,
    QueryName,
)
from probabilistic_model.benchmark.methods import (
    LearnSPNMethod,
    MethodName,
    StrudelMethod,
)
from probabilistic_model.benchmark.runner import BenchmarkRunner, RunStatus


# %% datasets in the cache


@pytest.fixture
def cache_directory(tmp_path: Path) -> Path:
    """
    A cache holding the three splits of a small binary dataset under the name of NLTCS.
    """
    generator = np.random.default_rng(0)
    for split, rows in zip(BinarySplit, (200, 50, 50)):
        bits = (generator.random((rows, 4)) < 0.4).astype(int)
        bits[:, 1] = bits[:, 0]
        path = tmp_path / "binary" / f"{BinaryDatasetName.NLTCS}.{split}.data"
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(path, bits, fmt="%d", delimiter=",")
    return tmp_path


@dataclass(kw_only=True)
class InMemoryMixedSource(MixedDatasetSource):
    """
    A mixed dataset whose rows are given instead of downloaded.
    """

    frame: pd.DataFrame
    """
    All rows of the dataset.
    """

    def raw_frame(self) -> pd.DataFrame:
        """
        :return: :attr:`frame`, instead of the published rows.
        """
        return self.frame


def test_binary_source_reads_the_cached_splits_as_symbolic_variables(cache_directory):
    """
    A cached binary dataset loads as binary symbolic variables encoded by hash.
    """
    dataset = BinaryDatasetSource(
        cache_directory=cache_directory, dataset=BinaryDatasetName.NLTCS
    ).load()
    assert (len(dataset.train), len(dataset.validation), len(dataset.test)) == (
        200,
        50,
        50,
    )
    assert dataset.is_binary
    encoded = dataset.encoded(frame=dataset.train)
    assert set(np.unique(encoded)) <= {float(hash("0")), float(hash("1"))}


def test_mixed_source_splits_and_standardizes_by_the_training_rows(tmp_path: Path):
    """
    A mixed dataset is split by the fractions and standardized by its training rows.
    """
    generator = np.random.default_rng(0)
    frame = pd.DataFrame(
        {
            "size": generator.normal(10.0, 3.0, 100),
            "kind": generator.choice(["a", "b"], 100),
        }
    )
    source = InMemoryMixedSource(
        cache_directory=tmp_path, dataset=MixedDatasetName.IRIS, frame=frame
    )
    dataset = source.load()
    assert len(dataset.test) == int(source.test_fraction * len(frame))
    assert len(dataset.validation) == int(source.validation_fraction * len(frame))
    assert dataset.train["size"].mean() == pytest.approx(0.0)
    assert dataset.train["size"].std() == pytest.approx(1.0)
    variable_types = {variable.name: type(variable) for variable in dataset.variables}
    assert variable_types == {"size": Continuous, "kind": Symbolic}


# %% measurements


def test_measurement_reports_every_query_and_capability(cache_directory):
    """
    A measurement times every query and checks every capability.
    """
    dataset = BinaryDatasetSource(
        cache_directory=cache_directory, dataset=BinaryDatasetName.NLTCS
    ).load()
    result = Measurement(dataset=dataset, method=LearnSPNMethod()).run()
    assert set(result.query_seconds) == {str(name) for name in QueryName}
    assert set(result.capabilities) == {str(name) for name in Capability}
    assert result.capabilities[str(Capability.MARGINAL)]
    assert result.test_unsupported_fraction == 0.0
    assert result.test_log_likelihood == pytest.approx(
        result.test_log_likelihood_of_supported_rows
    )


def test_strudel_circuit_answers_most_likely_assignments_exactly(cache_directory):
    """
    A deterministic Strudel circuit counts as answering most likely assignments exactly.
    """
    dataset = BinaryDatasetSource(
        cache_directory=cache_directory, dataset=BinaryDatasetName.NLTCS
    ).load()
    result = Measurement(dataset=dataset, method=StrudelMethod(maximum_splits=5)).run()
    assert result.capabilities[str(Capability.MOST_LIKELY_ASSIGNMENT)]


# %% runner


def test_runner_appends_one_record_per_run(cache_directory, tmp_path: Path):
    """
    The runner writes one finished record per dataset and method.
    """
    output = tmp_path / "results" / "run.jsonl"
    BenchmarkRunner(
        datasets=[str(BinaryDatasetName.NLTCS)],
        methods=[MethodName.LEARN_SPN],
        output=output,
        cache_directory=cache_directory,
    ).run()
    (record,) = [json.loads(line) for line in output.read_text().splitlines()]
    assert record["status"] == RunStatus.FINISHED
    assert record["method"] == MethodName.LEARN_SPN

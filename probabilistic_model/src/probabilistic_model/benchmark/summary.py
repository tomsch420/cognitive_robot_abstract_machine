"""
Summarizes the JSON lines files of benchmark runs into markdown tables: the test log-
likelihood of every method on every dataset, the average rank of every method, the cost
of learning, the speed of the queries and which queries every method answers exactly.
"""

from __future__ import annotations

# %% imports
import argparse
import json
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import numpy as np
import pandas as pd

from probabilistic_model.benchmark.datasets import BinaryDatasetName, MixedDatasetName
from probabilistic_model.benchmark.measurements import Capability, QueryName
from probabilistic_model.benchmark.runner import RunStatus


# %% columns


class ResultColumn(StrEnum):
    """
    The fields of a result record the summary reads.
    """

    DATASET = "dataset"
    METHOD = "method"
    STATUS = "status"
    TEST_LOG_LIKELIHOOD = "test_log_likelihood_of_supported_rows"
    UNSUPPORTED_FRACTION = "test_unsupported_fraction"
    TRAINING_SECONDS = "training_seconds"
    PEAK_MEMORY = "peak_memory_megabytes"
    PEAK_DEVICE_MEMORY = "peak_device_memory_megabytes"
    PARAMETERS = "number_of_parameters"
    QUERY_SECONDS = "query_seconds"
    CAPABILITIES = "capabilities"


# %% summary


@dataclass(kw_only=True)
class BenchmarkSummary:
    """
    The latest record of every method on every dataset, read from result files, and the
    tables made from them.
    """

    result_files: list[Path]
    """
    The JSON lines files, a later file's record replacing an earlier one's.
    """

    records: pd.DataFrame = field(init=False)
    """
    One row per dataset and method.
    """

    def __post_init__(self) -> None:
        """
        Read the records, keeping the last one of every dataset and method.
        """
        rows = [
            json.loads(line)
            for path in self.result_files
            for line in path.read_text().splitlines()
            if line.strip()
        ]
        self.records = (
            pd.DataFrame(rows)
            .drop_duplicates(
                subset=[ResultColumn.DATASET, ResultColumn.METHOD], keep="last"
            )
            .reset_index(drop=True)
        )

    def finished(self) -> pd.DataFrame:
        """
        :return: The records of the runs that finished.
        """
        return self.records[self.records[ResultColumn.STATUS] == RunStatus.FINISHED]

    def test_log_likelihoods(self, *, datasets: list[str]) -> pd.DataFrame:
        """
        :param datasets: The datasets of the table's rows.
        :return: The average test log-likelihood of the supported rows of every method
            on every dataset; missing where the run did not finish.
        """
        table = self.finished().pivot(
            index=ResultColumn.DATASET,
            columns=ResultColumn.METHOD,
            values=ResultColumn.TEST_LOG_LIKELIHOOD,
        )
        return table.reindex(
            [dataset for dataset in datasets if dataset in table.index]
        )

    @staticmethod
    def average_ranks(*, table: pd.DataFrame) -> pd.Series:
        """
        :param table: Test log-likelihoods, datasets by methods.
        :return: The average rank of every method over the datasets it finished on, one
            the best.
        """
        return table.rank(axis=1, ascending=False).mean().sort_values()

    def costs(self) -> pd.DataFrame:
        """
        :return: The median training time, peak memory, peak device memory and number of
            parameters of every method over the datasets it finished on.
        """
        return (
            self.finished()
            .groupby(ResultColumn.METHOD)[
                [
                    ResultColumn.TRAINING_SECONDS,
                    ResultColumn.PEAK_MEMORY,
                    ResultColumn.PEAK_DEVICE_MEMORY,
                    ResultColumn.PARAMETERS,
                ]
            ]
            .median()
            .sort_values(ResultColumn.TRAINING_SECONDS)
        )

    def query_speeds(self) -> pd.DataFrame:
        """
        :return: The median time in milliseconds of every query of every method.
        """
        finished = self.finished()
        speeds = pd.DataFrame(finished[ResultColumn.QUERY_SECONDS].tolist()) * 1000
        speeds[ResultColumn.METHOD] = finished[ResultColumn.METHOD].to_numpy()
        return speeds.groupby(ResultColumn.METHOD)[
            [str(name) for name in QueryName]
        ].median()

    def capabilities(self) -> pd.DataFrame:
        """
        :return: The fraction of the datasets on which every method's circuit answers
            every query of :class:`Capability` exactly.
        """
        finished = self.finished()
        flags = pd.DataFrame(finished[ResultColumn.CAPABILITIES].tolist()).astype(float)
        flags[ResultColumn.METHOD] = finished[ResultColumn.METHOD].to_numpy()
        return flags.groupby(ResultColumn.METHOD)[
            [str(name) for name in Capability]
        ].mean()

    def failures(self) -> pd.DataFrame:
        """
        :return: The dataset, method and status of every run that did not finish.
        """
        return self.records[self.records[ResultColumn.STATUS] != RunStatus.FINISHED][
            [ResultColumn.DATASET, ResultColumn.METHOD, ResultColumn.STATUS]
        ]

    def to_markdown(self) -> str:
        """
        :return: Every table of the summary as markdown.
        """
        binary = self.test_log_likelihoods(
            datasets=[str(name) for name in BinaryDatasetName]
        )
        mixed = self.test_log_likelihoods(
            datasets=[str(name) for name in MixedDatasetName]
        )
        sections = [
            ("Test log-likelihood, binary datasets", binary.round(2)),
            (
                "Average rank, binary datasets",
                self.average_ranks(table=binary).round(2),
            ),
            ("Test log-likelihood, UCI datasets", mixed.round(2)),
            ("Average rank, UCI datasets", self.average_ranks(table=mixed).round(2)),
            ("Median cost of learning", self.costs().round(2)),
            ("Median query time in milliseconds", self.query_speeds().round(2)),
            ("Fraction of datasets answered exactly", self.capabilities().round(2)),
            ("Runs that did not finish", self.failures()),
        ]
        return "\n\n".join(
            f"## {title}\n\n{frame.to_markdown()}" for title, frame in sections
        )


# %% command line


def main() -> None:
    """
    Parse the command line and print the summary.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_files", nargs="+", type=Path)
    print(BenchmarkSummary(result_files=parser.parse_args().result_files).to_markdown())


if __name__ == "__main__":
    main()

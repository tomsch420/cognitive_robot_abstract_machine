"""
Runs the circuit learning benchmark: every chosen method on every chosen dataset, each
in a process of its own so that its peak memory is its own, and appends one JSON line
per run to a results file.
"""

from __future__ import annotations

# %% imports
import argparse
import dataclasses
import json
import multiprocessing
import os
import sys
import time
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

import jax

from probabilistic_model.benchmark.datasets import (
    BinaryDatasetName,
    BinaryDatasetSource,
    DatasetSource,
    MixedDatasetName,
    MixedDatasetSource,
)
from probabilistic_model.benchmark.measurements import Measurement
from probabilistic_model.benchmark.methods import BenchmarkMethods, MethodName


# %% runs


class RunStatus(StrEnum):
    """
    How a run ended.
    """

    FINISHED = "finished"
    FAILED = "failed"
    TIMED_OUT = "timed_out"


class FloatingPointType(StrEnum):
    """
    The floating point type jax computes a run in.
    """

    FLOAT32 = "float32"
    FLOAT64 = "float64"


@dataclass(kw_only=True)
class BenchmarkRun:
    """
    One method on one dataset, run in a process of its own.
    """

    dataset: str
    """
    The name of the dataset, a :class:`BinaryDatasetName` or :class:`MixedDatasetName`.
    """

    method: MethodName
    """
    The method.
    """

    cache_directory: Path
    """
    The directory of the downloaded datasets.
    """

    result_path: Path
    """
    The file the process writes its result to.
    """

    log_path: Path
    """
    The file the process writes its output and errors to.
    """

    floating_point_type: FloatingPointType
    """
    The floating point type the circuit is learned and measured in.
    """

    def source(self) -> DatasetSource:
        """
        :return: The source of the dataset.
        """
        if self.dataset in BinaryDatasetName.__members__.values():
            return BinaryDatasetSource(
                cache_directory=self.cache_directory,
                dataset=BinaryDatasetName(self.dataset),
            )
        return MixedDatasetSource(
            cache_directory=self.cache_directory, dataset=MixedDatasetName(self.dataset)
        )

    def execute(self) -> None:
        """
        Learn and measure, and write the result to :attr:`result_path`.
        """
        log = os.open(self.log_path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC)
        os.dup2(log, sys.stdout.fileno())
        os.dup2(log, sys.stderr.fileno())
        jax.config.update(
            "jax_enable_x64", self.floating_point_type is FloatingPointType.FLOAT64
        )
        result = Measurement(
            dataset=self.source().load(),
            method=BenchmarkMethods().find(name=self.method),
        ).run()
        self.result_path.write_text(json.dumps(dataclasses.asdict(result)))


@dataclass(kw_only=True)
class BenchmarkRunner:
    """
    Runs every method on every dataset and appends the results to a JSON lines file.
    """

    datasets: list[str]
    """
    The names of the datasets.
    """

    methods: list[MethodName]
    """
    The methods.
    """

    output: Path
    """
    The JSON lines file the results are appended to.
    """

    cache_directory: Path
    """
    The directory of the downloaded datasets.
    """

    timeout_seconds: float = 3600.0
    """
    The time after which a run is stopped.
    """

    floating_point_type: FloatingPointType = FloatingPointType.FLOAT64
    """
    The floating point type every run computes in.
    """

    log_tail_characters: int = 2000
    """
    How much of the end of a failed run's output its record keeps.
    """

    def run(self) -> None:
        """
        Run every method on every dataset, one process at a time.
        """
        context = multiprocessing.get_context("spawn")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        for dataset in self.datasets:
            for method in self.methods:
                result_path = self.output.with_suffix(f".{dataset}.{method}.json")
                result_path.unlink(missing_ok=True)
                log_path = self.output.with_suffix(f".{dataset}.{method}.log")
                run = BenchmarkRun(
                    dataset=dataset,
                    method=method,
                    cache_directory=self.cache_directory,
                    result_path=result_path,
                    log_path=log_path,
                    floating_point_type=self.floating_point_type,
                )
                start = time.perf_counter()
                process = context.Process(target=run.execute)
                process.start()
                process.join(self.timeout_seconds)
                status = RunStatus.FINISHED
                if process.is_alive():
                    process.kill()
                    process.join()
                    status = RunStatus.TIMED_OUT
                elif process.exitcode != 0 or not result_path.exists():
                    status = RunStatus.FAILED
                record = {
                    "dataset": dataset,
                    "method": str(method),
                    "status": str(status),
                    "floating_point_type": str(self.floating_point_type),
                    "wall_seconds": time.perf_counter() - start,
                }
                if status is RunStatus.FINISHED:
                    record.update(json.loads(result_path.read_text()))
                    result_path.unlink()
                else:
                    record["log_tail"] = log_path.read_text()[
                        -self.log_tail_characters :
                    ]
                with self.output.open("a") as file:
                    file.write(json.dumps(record) + "\n")
                print(json.dumps(record), flush=True)


# %% command line


def main() -> None:
    """
    Parse the command line and run the benchmark.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--datasets",
        nargs="+",
        default=[str(name) for name in BinaryDatasetName]
        + [str(name) for name in MixedDatasetName],
    )
    parser.add_argument(
        "--methods", nargs="+", default=[str(name) for name in MethodName]
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--cache-directory",
        type=Path,
        default=Path.home() / ".cache" / "probabilistic_model" / "benchmark",
    )
    parser.add_argument("--timeout-seconds", type=float, default=3600.0)
    parser.add_argument(
        "--floating-point-type",
        type=FloatingPointType,
        choices=list(FloatingPointType),
        default=FloatingPointType.FLOAT64,
    )
    arguments = parser.parse_args()
    BenchmarkRunner(
        datasets=arguments.datasets,
        methods=[MethodName(name) for name in arguments.methods],
        output=arguments.output,
        cache_directory=arguments.cache_directory,
        timeout_seconds=arguments.timeout_seconds,
        floating_point_type=arguments.floating_point_type,
    ).run()


if __name__ == "__main__":
    main()

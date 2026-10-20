"""
Benchmarking the PR2 on RoboCasa's counter to sink task over kitchen layouts and seeds.

Every episode runs in a process of its own, which writes a record of how far it got and
how it ended, so that an episode that crashes or runs out of memory costs only itself.
The PR2 grasps the target object where the grasp model learned for its RoboCasa
category expects to lift it, and by the grasps its annotation offers by itself where no
model was learned.

Run with (the ``experiments`` package must be importable, and
:attr:`~experiments.grasp_learning.database.GraspDatabaseEnvironmentVariable.URI` set to
the database of the grasp attempts)::

    python -m experiments.robocasa_task_execution.benchmark \\
        --layouts LAYOUT001 LAYOUT002 --seeds 0 1 --results /tmp/benchmark
"""

from __future__ import annotations

import argparse
import logging
import subprocess
import sys
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from typing_extensions import Dict, List

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

from experiments.robocasa_task_execution import benchmark_episode
from experiments.robocasa_task_execution.benchmark_episode import EpisodeRun
from experiments.robocasa_task_execution.benchmark_records import (
    BenchmarkEpisode,
    EpisodeRecord,
)
from experiments.robocasa_task_execution.counter_to_sink import EpisodeStage

logger = logging.getLogger(__name__)


# %% the benchmark


@dataclass
class CounterToSinkBenchmark:
    """
    Runs episodes of the counter to sink task, each in a process of its own, and records
    how each ended.
    """

    episodes: List[BenchmarkEpisode]
    """
    The episodes to run.
    """

    results: Path
    """
    The directory the records and the logs of the episodes are written to.
    """

    workers: int = 4
    """
    How many episodes run at the same time.
    """

    physics: bool = EpisodeRun.physics
    """
    Whether the episodes are performed in MuJoCo physics instead of kinematically.
    """

    collision_avoidance: bool = EpisodeRun.collision_avoidance
    """
    Whether the PR2 avoids colliding with the kitchen and with itself.
    """

    learned_grasps: bool = EpisodeRun.learned_grasps
    """
    Whether the PR2 tries grasps from the models learned for the target objects'
    categories.
    """

    def run(self) -> List[EpisodeRecord]:
        """
        :return: The record of every episode, in the order of :attr:`episodes`; an
            episode recorded as finished before is not run again.
        """
        self.results.mkdir(parents=True, exist_ok=True)
        with ThreadPoolExecutor(self.workers) as executor:
            return list(executor.map(self.run_episode, self.episodes))

    def run_episode(self, episode: BenchmarkEpisode) -> EpisodeRecord:
        """
        :param episode: The episode to run.
        :return: Its record, read from an earlier run if it finished there.
        """
        record_file = self.record_file(episode)
        earlier = EpisodeRecord.read(record_file) if record_file.exists() else None
        if earlier is not None and earlier.exit_code is not None:
            return earlier
        record_file.unlink(missing_ok=True)
        with record_file.with_suffix(".log").open("w") as log:
            process = subprocess.run(
                self.episode_command(episode),
                stdout=log,
                stderr=subprocess.STDOUT,
            )
        record = self.finished_record(episode, process.returncode)
        logger.info(
            "%s seed %d: %s after %s, %.0f s",
            episode.layout.name,
            episode.seed,
            "succeeded" if record.succeeded else "failed",
            record.stage,
            record.seconds,
        )
        return record

    def episode_command(self, episode: BenchmarkEpisode) -> List[str]:
        """
        :param episode: The episode to run.
        :return: The command performing it in a process of its own.
        """
        return [
            sys.executable,
            "-m",
            benchmark_episode.__spec__.name,
            "--layout",
            episode.layout.name,
            "--seed",
            str(episode.seed),
            "--record",
            str(self.record_file(episode)),
            "--physics" if self.physics else "--no-physics",
            (
                "--collision-avoidance"
                if self.collision_avoidance
                else "--no-collision-avoidance"
            ),
            "--learned-grasps" if self.learned_grasps else "--no-learned-grasps",
        ]

    def record_file(self, episode: BenchmarkEpisode) -> Path:
        """
        :param episode: An episode of the benchmark.
        :return: The file its record is written to.
        """
        return self.results / f"{episode.layout.name}_seed_{episode.seed}.json"

    def finished_record(
        self, episode: BenchmarkEpisode, exit_code: int
    ) -> EpisodeRecord:
        """
        :param episode: An episode whose process ended.
        :param exit_code: The exit code of the process.
        :return: The record the process wrote, or one of an episode that never got past
            composing if it wrote none, with the exit code; also written back.
        """
        record_file = self.record_file(episode)
        record = (
            EpisodeRecord.read(record_file)
            if record_file.exists()
            else EpisodeRecord(episode=episode)
        )
        record.exit_code = exit_code
        record.write(record_file)
        return record


# %% summarizing


@dataclass
class BenchmarkSummary:
    """
    How often the episodes of a benchmark succeeded, and where the others ended.
    """

    records: List[EpisodeRecord]
    """
    The records of the episodes.
    """

    @property
    def success_rate(self) -> float:
        """
        The share of the episodes that succeeded.
        """
        return sum(record.succeeded for record in self.records) / len(self.records)

    def unsuccessful_by_stage(self) -> Dict[EpisodeStage, int]:
        """
        :return: How many unsuccessful episodes ended in each stage; an episode that
            finished but missed the sink ended in :attr:`EpisodeStage.FINISHED`.
        """
        return dict(
            Counter(record.stage for record in self.records if not record.succeeded)
        )

    def success_rate_by_layout(self) -> Dict[LayoutType, float]:
        """
        :return: The share of the episodes in each layout that succeeded.
        """
        layouts = {record.episode.layout for record in self.records}
        return {
            layout: BenchmarkSummary(
                records=[
                    record for record in self.records if record.episode.layout == layout
                ]
            ).success_rate
            for layout in sorted(layouts)
        }

    def report(self) -> str:
        """
        :return: The summary as text, one line per figure.
        """
        lines = [
            f"{len(self.records)} episodes, success rate {self.success_rate:.2f}",
            *(
                f"  {layout.name}: {rate:.2f}"
                for layout, rate in self.success_rate_by_layout().items()
            ),
            "unsuccessful episodes by the stage they ended in:",
            *(
                f"  {stage}: {count}"
                for stage, count in self.unsuccessful_by_stage().items()
            ),
            "failures: "
            + ", ".join(
                f"{failure} {count}"
                for failure, count in Counter(
                    record.failure for record in self.records if record.failure
                ).items()
            ),
        ]
        return "\n".join(lines)


# %% the command line


def _parse_arguments() -> argparse.Namespace:
    """
    :return: The parsed command line choosing the episodes and how they are performed.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--layouts",
        nargs="+",
        required=True,
        choices=[layout.name for layout in LayoutType],
    )
    parser.add_argument("--seeds", nargs="+", type=int, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=CounterToSinkBenchmark.workers)
    for flag, default in (
        ("--physics", CounterToSinkBenchmark.physics),
        ("--collision-avoidance", CounterToSinkBenchmark.collision_avoidance),
        ("--learned-grasps", CounterToSinkBenchmark.learned_grasps),
    ):
        parser.add_argument(
            flag, action=argparse.BooleanOptionalAction, default=default
        )
    return parser.parse_args()


def main() -> None:
    """
    Run every episode of the benchmark and write its summary.
    """
    logging.basicConfig(level=logging.INFO, force=True)
    arguments = _parse_arguments()
    benchmark = CounterToSinkBenchmark(
        episodes=[
            BenchmarkEpisode(layout=LayoutType[layout], seed=seed)
            for layout in arguments.layouts
            for seed in arguments.seeds
        ],
        results=arguments.results,
        workers=arguments.workers,
        physics=arguments.physics,
        collision_avoidance=arguments.collision_avoidance,
        learned_grasps=arguments.learned_grasps,
    )
    summary = BenchmarkSummary(records=benchmark.run())
    (arguments.results / "summary.txt").write_text(summary.report())
    logger.info("\n%s", summary.report())


if __name__ == "__main__":
    main()

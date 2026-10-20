"""
One episode of the counter to sink benchmark, performed in a process of its own, which
writes the episode's record also when the episode ends in an exception.

Run with (see :mod:`experiments.robocasa_task_execution.benchmark`)::

    python -m experiments.robocasa_task_execution.benchmark_episode \\
        --layout LAYOUT001 --seed 0 --record /tmp/LAYOUT001_seed_0.json
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import TracebackType

from typing_extensions import Type

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

from experiments.grasp_learning.category_models import CategoryGraspModels
from experiments.grasp_learning.database import GraspDatabase
from experiments.robocasa_task_execution.benchmark_records import (
    BenchmarkEpisode,
    EpisodeRecord,
)
from experiments.robocasa_task_execution.counter_to_sink import CounterToSinkEpisode

# %% performing the episode


@dataclass
class EpisodeRun:
    """
    Performs one episode and writes its record, also when the episode ends in an
    exception.
    """

    record: EpisodeRecord
    """
    The record of the episode, filled in as it goes.
    """

    record_file: Path
    """
    The file the record is written to.
    """

    physics: bool = True
    """
    Whether the episode is performed in MuJoCo physics instead of kinematically.
    """

    collision_avoidance: bool = True
    """
    Whether the PR2 avoids colliding with the kitchen and with itself.
    """

    learned_grasps: bool = True
    """
    Whether the PR2 tries grasps from the model learned for the target object's
    category.
    """

    episode: CounterToSinkEpisode | None = field(default=None, init=False)
    """
    The composed episode; ``None`` while it is composed.
    """

    start: float = field(default_factory=time.monotonic, init=False)
    """
    When the run began, on the monotonic clock.
    """

    def perform(self) -> None:
        """
        Compose and perform the episode, and write its record.
        """
        sys.excepthook = self.record_failure
        self.episode = CounterToSinkEpisode.compose(
            self.record.episode.layout,
            StyleType.STYLE001,
            seed=self.record.episode.seed,
        )
        self.record.category = self.episode.target_category
        grasp_model = (
            CategoryGraspModels(database=GraspDatabase.from_environment()).model_for(
                self.record.category
            )
            if self.learned_grasps
            else None
        )
        self.record.grasped_by_learned_model = grasp_model is not None
        self.write()
        if not self.physics:
            self.record.outcome = self.episode.perform(
                sampling_seed=self.record.episode.seed,
                collision_avoidance=self.collision_avoidance,
                grasp_model=grasp_model,
            )
            self.write()
            return
        simulation = self.episode.start_physical_simulation()
        self.record.outcome = self.episode.perform_in_physics(
            simulation,
            sampling_seed=self.record.episode.seed,
            collision_avoidance=self.collision_avoidance,
            grasp_model=grasp_model,
        )
        simulation.stop_simulation()
        self.write()

    def record_failure(
        self,
        exception_type: Type[BaseException],
        exception: BaseException,
        traceback: TracebackType | None,
    ) -> None:
        """
        Record the exception that ended the episode, then report it as Python does.
        """
        self.record.failure = exception_type.__name__
        self.write()
        sys.__excepthook__(exception_type, exception, traceback)

    def write(self) -> None:
        """
        Write the record with the stage the episode reached and the time it took so far.
        """
        if self.episode is not None:
            self.record.stage = self.episode.stage
        self.record.seconds = time.monotonic() - self.start
        self.record.write(self.record_file)


# %% the command line


def _parse_arguments() -> argparse.Namespace:
    """
    :return: The parsed command line choosing the episode and how it is performed.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--layout", required=True, choices=[layout.name for layout in LayoutType]
    )
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--record", type=Path, required=True)
    for flag, default in (
        ("--physics", EpisodeRun.physics),
        ("--collision-avoidance", EpisodeRun.collision_avoidance),
        ("--learned-grasps", EpisodeRun.learned_grasps),
    ):
        parser.add_argument(
            flag, action=argparse.BooleanOptionalAction, default=default
        )
    return parser.parse_args()


def main() -> None:
    """
    Perform one episode of the benchmark and write its record.
    """
    logging.basicConfig(level=logging.INFO, force=True)
    arguments = _parse_arguments()
    EpisodeRun(
        record=EpisodeRecord(
            episode=BenchmarkEpisode(
                layout=LayoutType[arguments.layout], seed=arguments.seed
            )
        ),
        record_file=arguments.record,
        physics=arguments.physics,
        collision_avoidance=arguments.collision_avoidance,
        learned_grasps=arguments.learned_grasps,
    ).perform()


if __name__ == "__main__":
    main()

"""
What is recorded of an episode of the counter to sink benchmark.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

from experiments.robocasa_task_execution.counter_to_sink import (
    CounterToSinkOutcome,
    EpisodeStage,
)
from krrood.adapters.json_serializer import from_json, to_json

# %% what is benchmarked


@dataclass(frozen=True)
class BenchmarkEpisode:
    """
    One episode of the benchmark: the kitchen it is composed in and the seed RoboCasa
    samples its objects with.
    """

    layout: LayoutType
    """
    The kitchen layout.
    """

    seed: int
    """
    The seed RoboCasa samples the objects and their placements with, and the robot its
    standing poses.
    """


# %% what is recorded of an episode


@dataclass
class EpisodeRecord:
    """
    How far one episode got and how it ended.
    """

    episode: BenchmarkEpisode
    """
    The episode.
    """

    stage: EpisodeStage = EpisodeStage.COMPOSING
    """
    The last stage the episode began.
    """

    category: str | None = None
    """
    The RoboCasa category of the target object; ``None`` before it is known.
    """

    grasped_by_learned_model: bool | None = None
    """
    Whether the robot tried grasps from a model learned for the target object's
    category; ``None`` before it is known.
    """

    outcome: CounterToSinkOutcome | None = None
    """
    How the episode ended; ``None`` if it did not finish.
    """

    failure: str | None = None
    """
    The name of the exception that ended the episode early; ``None`` without one.
    """

    seconds: float = 0.0
    """
    How long the episode ran, composing included.
    """

    exit_code: int | None = None
    """
    The exit code of the episode's process; ``None`` while it runs.
    """

    @property
    def succeeded(self) -> bool:
        """
        Whether the episode finished with the target object let go of in the sink.
        """
        return self.outcome is not None and self.outcome.succeeded

    def write(self, path: Path) -> None:
        """
        :param path: The file to write the record to.
        """
        path.write_text(json.dumps(to_json(self), indent=2))

    @classmethod
    def read(cls, path: Path) -> EpisodeRecord:
        """
        :param path: A file a record was written to.
        :return: The record.
        """
        return from_json(json.loads(path.read_text()))

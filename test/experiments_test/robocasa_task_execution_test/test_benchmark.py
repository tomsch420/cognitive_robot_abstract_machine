"""
Benchmarking the PR2 on RoboCasa's counter to sink task: what is recorded of each
episode and how the episodes are summarized.
"""

from __future__ import annotations

import pytest

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    pytest.importorskip("robocasa", reason="robocasa is not installed")
pytest.importorskip("robosuite", reason="robosuite is not installed")

from experiments.robocasa_task_execution.benchmark import (
    BenchmarkSummary,
    CounterToSinkBenchmark,
)
from experiments.robocasa_task_execution.benchmark_records import (
    BenchmarkEpisode,
    EpisodeRecord,
)
from experiments.robocasa_task_execution.counter_to_sink import (
    CounterToSinkOutcome,
    EpisodeStage,
)
from robocasa.models.scenes.scene_registry import LayoutType

# %% records of episodes

SUCCEEDED = CounterToSinkOutcome(object_in_sink=True, object_released=True)
"""
How an episode ends that put the object into the sink.
"""

MISSED_THE_SINK = CounterToSinkOutcome(object_in_sink=False, object_released=True)
"""
How an episode ends that let go of the object outside the sink.
"""


def record_of(
    stage: EpisodeStage,
    outcome: CounterToSinkOutcome | None = None,
    layout: LayoutType = LayoutType.LAYOUT001,
    seed: int = 0,
) -> EpisodeRecord:
    """
    :return: The record of an episode of an apple that reached ``stage`` and ended
        with ``outcome``.
    """
    return EpisodeRecord(
        episode=BenchmarkEpisode(layout=layout, seed=seed),
        stage=stage,
        category="apple",
        grasped_by_learned_model=True,
        outcome=outcome,
    )


def test_an_episode_record_reads_back_as_it_was_written(tmp_path):
    record = record_of(EpisodeStage.PLACING, layout=LayoutType.LAYOUT002, seed=3)
    record.failure = "CandidateLimitReached"
    record.seconds = 12.5
    record.exit_code = 1
    path = tmp_path / "record.json"

    record.write(path)

    assert EpisodeRecord.read(path) == record


def test_an_episode_whose_process_left_no_record_is_recorded_with_its_exit_code(
    tmp_path,
):
    episode = BenchmarkEpisode(layout=LayoutType.LAYOUT003, seed=2)
    benchmark = CounterToSinkBenchmark(episodes=[episode], results=tmp_path)

    record = benchmark.finished_record(episode, exit_code=-9)

    assert record == EpisodeRecord(episode=episode, exit_code=-9)
    assert EpisodeRecord.read(benchmark.record_file(episode)) == record


def test_an_episode_that_wrote_its_record_keeps_it_with_its_exit_code(tmp_path):
    written = record_of(EpisodeStage.FINISHED, SUCCEEDED)
    benchmark = CounterToSinkBenchmark(episodes=[written.episode], results=tmp_path)
    written.write(benchmark.record_file(written.episode))

    record = benchmark.finished_record(written.episode, exit_code=0)

    written.exit_code = 0
    assert record == written


# %% summarizing


def test_only_an_episode_that_finished_with_the_object_in_the_sink_succeeded():
    assert record_of(EpisodeStage.FINISHED, SUCCEEDED).succeeded
    assert not record_of(EpisodeStage.FINISHED, MISSED_THE_SINK).succeeded
    assert not record_of(EpisodeStage.PICKING_UP).succeeded


def test_a_summary_counts_the_successes_and_where_the_other_episodes_ended():
    records = [
        record_of(EpisodeStage.FINISHED, SUCCEEDED, seed=0),
        record_of(EpisodeStage.FINISHED, MISSED_THE_SINK, seed=1),
        record_of(EpisodeStage.PICKING_UP, seed=2),
        record_of(EpisodeStage.PICKING_UP, layout=LayoutType.LAYOUT002),
    ]

    summary = BenchmarkSummary(records=records)

    assert summary.success_rate == 1 / 4
    assert summary.unsuccessful_by_stage() == {
        EpisodeStage.FINISHED: 1,
        EpisodeStage.PICKING_UP: 2,
    }
    assert summary.success_rate_by_layout() == {
        LayoutType.LAYOUT001: 1 / 3,
        LayoutType.LAYOUT002: 0.0,
    }

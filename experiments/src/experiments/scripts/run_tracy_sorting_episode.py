"""
Runs one episode of :class:`~experiments.montessori.scenarios.TracySortsAPiece`,
records it as one :class:`~experiments.episodes.episode.Episode` (through ORMatic, to a
Postgres database), films it, and tags the written video with the episode's own
identifier so a viewer of the video can find its row.

Meant to be invoked once per episode by a driver that loops it (e.g. a shell loop), so
that one episode's own crash -- MuJoCo's offscreen renderer is not guaranteed leak-free
across many sequential headless contexts in one process -- never takes the rest of a
run's episodes down with it.

Run with (the ``iai_tracy_description`` ROS package must be built and sourced)::

    python -m experiments.scripts.run_tracy_sorting_episode
"""

from __future__ import annotations

import argparse
import logging
import subprocess
import sys
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


def main(argument_list: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--database-uri", default=None)
    parser.add_argument("--artifact-directory", default=None)
    arguments = parser.parse_args(argument_list)

    # Imported here, not at module level: importing the CRAM stack (coraplex, giskardpy,
    # semantic_digital_twin, ...) costs the best part of a minute, which a caller only
    # wanting --help should not pay.
    import imageio_ffmpeg
    from coraplex.datastructures.enums import Arms

    from experiments.episodes.artifacts import ArtifactDirectory
    from experiments.episodes.episode import Episode
    from experiments.episodes.recording import EpisodeRecording, open_recording
    from experiments.montessori.pieces import KNOWN_PIECE_BY_CATEGORY
    from experiments.montessori.results_database import ConfiguredDatabase, ResultsDatabase
    from experiments.montessori.semantics import MontessoriShapeCategory
    from experiments.montessori.scenarios import PieceLayout, PiecePlacement, TracySortsAPiece
    from experiments.tracy_experiments.montessori.world import (
        SHAPE_ROW_START_Y,
        SHAPE_ROW_X,
        TracyMontessoriWorldBuilder,
    )
    from semantic_digital_twin.adapters.mujoco_video_recording import RecordedVideo

    database = ConfiguredDatabase.resolve_reachable(arguments.database_uri)
    if database.fell_back_from is not None:
        logger.warning("%s", database.fell_back_from)
    logger.info("%s", database.describe())
    results_database = ResultsDatabase(uri=database.uri)
    records_trials = open_recording(results_database)

    artifact_directory = ArtifactDirectory(
        **(
            {}
            if arguments.artifact_directory is None
            else {"path": Path(arguments.artifact_directory)}
        )
    )

    layout = PieceLayout(
        placements=[
            PiecePlacement(
                piece=KNOWN_PIECE_BY_CATEGORY[MontessoriShapeCategory.CUBE],
                x=SHAPE_ROW_X,
                y=SHAPE_ROW_START_Y,
                yaw=0.0,
            )
        ]
    )
    scenario = TracySortsAPiece(
        layout=layout,
        world_builder=TracyMontessoriWorldBuilder(),
        sorted_category=MontessoriShapeCategory.CUBE,
        filmed=True,
        # The production arm (THE_ARM_THAT_SORTS, the right one) does not reliably
        # converge its reach at this board position in simulation; the left arm's reach
        # here is proven (see montessori_demo_mujoco.py). Corpus generation wants
        # working episodes, not a repro of that open tuning gap, so it overrides the
        # production default.
        arm=Arms.LEFT,
        # Filming this at the usual 15fps is dominated by rendering, not physics: a
        # single episode costs 20+ minutes of wall clock almost entirely on redundant
        # frames. Roughly one frame per simulated second is still enough to see what
        # happened without paying for smoothness nobody asked for.
        video_frames_per_second=1,
    )
    # episode.world and episode.ticks are filled in automatically once the trial
    # finishes (see EpisodeRecording.trial_finished, which reads them off the scenario
    # itself) -- not set here, since the world doesn't exist until runner.run(scenario)
    # has already built it, and that same call is also what records the trial; setting
    # them any later than that would miss the recording entirely.
    episode = Episode.from_run(scenario=scenario)
    runner = EpisodeRecording(repetitions=1, episode=episode, records_trials=records_trials)

    try:
        report = runner.run(scenario)
        outcome = report.trials[0].outcome
    finally:
        if scenario.simulation is not None:
            scenario.simulation.stop()
        records_trials.close()

    logger.info("Episode %s: %s", episode.identifier, outcome.value)

    recording = None if scenario.simulation is None else scenario.simulation.recording
    if recording is not None and recording.frame_count > 0:
        video = RecordedVideo(
            frames=recording.frames, frames_per_second=recording.frames_per_second
        )
        artifacts = artifact_directory.open_for(episode)
        video_path = artifacts.keep_video(video)
        _tag_video_with_its_episode(
            video_path, episode, outcome, imageio_ffmpeg.get_ffmpeg_exe()
        )
        logger.info("Video: %s (%d frames)", video_path, len(video.frames))
    else:
        logger.warning("Episode %s kept no video.", episode.identifier)

    print(episode.identifier)
    return 0


def _tag_video_with_its_episode(video_path: Path, episode, outcome, ffmpeg_exe: str) -> None:
    """
    Embed the episode this video belongs to as the video's own ``comment`` metadata tag,
    so a viewer of the video (not just a reader of the database) can find its row.

    :param video_path: The freshly written video to tag.
    :param episode: The episode it is a recording of.
    :param outcome: How the episode's trial ended.
    :param ffmpeg_exe: Path to the ffmpeg binary to tag it with.
    """
    tagged_path = video_path.with_suffix(".tagged.mp4")
    comment = (
        "ormatic_episode_id=%s;scenario=%s;outcome=%s;recorded_at=%s"
        % (
            episode.identifier,
            episode.scenario_name,
            outcome.value,
            episode.recorded_at.isoformat(),
        )
    )
    subprocess.run(
        [
            ffmpeg_exe,
            "-y",
            "-i",
            str(video_path),
            "-c",
            "copy",
            "-metadata",
            "comment=%s" % comment,
            "-metadata",
            "title=TracySortsAPiece episode %s" % episode.identifier,
            str(tagged_path),
        ],
        check=True,
        capture_output=True,
    )
    tagged_path.replace(video_path)


if __name__ == "__main__":
    sys.exit(main())

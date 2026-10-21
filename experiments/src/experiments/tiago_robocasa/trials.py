"""
Running one of Tiago's RoboCasa tasks over freshly composed kitchens.

A task is tried again in a new kitchen until it comes off: what RoboCasa samples decides
where the object lies and what stands around it, and so whether Tiago can stand where
the task needs it at all.
"""

from __future__ import annotations

import argparse
import logging
import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import mujoco

from experiments.scene_recording import SceneRecording
from experiments.tiago_robocasa.robocasa_task import TiagoRoboCasaTask
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from typing_extensions import Type

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

logger = logging.getLogger(__name__)


# %% how an episode is set up


@dataclass
class EpisodeSettings:
    """
    How each episode of a run is set up.

    What is left as ``None`` is left as the task itself has it.
    """

    recording: SceneRecording | None = None
    """
    Where the episode is recorded to, or nothing to run it unrecorded.
    """

    headless: bool = True
    """
    Whether the simulation runs without a viewer window.
    """

    noslip_iterations: int = 10
    """
    How many passes MuJoCo spends removing the slip its soft contacts leave.
    """

    friction_cone: Any = mujoco.mjtCone.mjCONE_ELLIPTIC
    """
    How MuJoCo bounds a contact's friction.
    """

    works_with_the_left_arm: bool = False
    """
    Which of Tiago's arms does the work.
    """

    collision_avoidance: bool = True
    """
    Whether the arm is planned around what stands in its way.
    """

    seed: int | None = None
    """
    The seed RoboCasa samples the kitchen with.
    """

    reach: float | None = None
    """
    How far in front of its base Tiago works.
    """

    grasp_pitch: float | None = None
    """
    How far from level the hand comes down onto what it takes hold of, in radians.
    """

    counter_height: float | None = None
    """
    What the kitchen's counters are lowered to, in metres.
    """

    food_category: str | None = None
    """
    Which kind of object RoboCasa is to sample for a task that moves one, or nothing to
    let it sample whatever it likes.
    """

    varies_the_layout: bool = False
    """
    Whether each attempt composes the next of RoboCasa's kitchen layouts, rather than
    the one the task has: what a task needs, a microwave on the counter rather than
    over the hob, say, is a matter of the layout, which a new seed does not change.
    """

    first_layout: int = 1
    """
    Which of RoboCasa's kitchen layouts the first attempt composes, where each attempt
    composes the next.
    """

    def recording_of(self, attempt: int) -> SceneRecording | None:
        """
        :param attempt: Which attempt this is.
        :return: Where that attempt is recorded to: a file of its own, named after the
            attempt, so that a kitchen the task did not come off in can be looked at
            afterwards.
        """
        if self.recording is None:
            return None
        path = self.recording.path
        return replace(
            self.recording, path=path.with_stem(f"{path.stem}_kitchen{attempt}")
        )

    def episode_of(
        self, task: Type[TiagoRoboCasaTask], attempt: int = 1
    ) -> TiagoRoboCasaTask:
        """
        :param task: The task to set an episode of up.
        :param attempt: Which attempt this is, which seeds the kitchen where the run
            does not ask for a seed of its own: RoboCasa hands back the same kitchen
            often enough that a run otherwise spends its attempts on one it cannot work
            in at all.
        :return: The episode.
        """
        episode = task(
            recording=self.recording_of(attempt),
            headless=self.headless,
            noslip_iterations=self.noslip_iterations,
            friction_cone=self.friction_cone,
            works_with_the_left_arm=self.works_with_the_left_arm,
            collision_avoidance=self.collision_avoidance,
        )
        episode.seed = self.seed if self.seed is not None else attempt
        if self.reach is not None:
            episode.reach = self.reach
        if self.grasp_pitch is not None:
            episode.grasp_pitch = self.grasp_pitch
        if self.counter_height is not None:
            episode.counter_height = self.counter_height
        if self.food_category is not None:
            episode.food_category = self.food_category
        if self.varies_the_layout:
            layout = self.first_layout + attempt - 1
            episode.layout = LayoutType[f"LAYOUT{layout:03d}"]
        return episode


# %% running the episodes


@dataclass
class Trials:
    """
    One of Tiago's RoboCasa tasks, tried over one kitchen after another.
    """

    task: Type[TiagoRoboCasaTask]
    """
    The task being tried.
    """

    settings: EpisodeSettings
    """
    How each episode is set up.
    """

    attempts: int = 12
    """
    How many kitchens are tried at most.
    """

    tries_every_kitchen: bool = False
    """
    Whether every kitchen is tried, to count how many the task comes off in, rather than
    stopping at the first it does.
    """

    done: int = field(init=False, default=0)
    """
    In how many kitchens the task came off.
    """

    def run(self) -> None:
        """
        Try the task until it comes off, or until every kitchen asked for has been
        tried, reporting what became of each.
        """
        for attempt in range(1, self.attempts + 1):
            episode = self.settings.episode_of(self.task, attempt)
            self.try_in_a_new_kitchen(attempt, episode)
            if self.done and not self.tries_every_kitchen:
                break
        self.report()

    def try_in_a_new_kitchen(self, attempt: int, episode: TiagoRoboCasaTask) -> None:
        """
        Compose a kitchen and carry the task out in it, counting it where it came off.

        A kitchen Tiago cannot work in is this experiment's subject rather than its
        error, so what the attempt raises is reported, along with how far the scene got,
        and the next kitchen is tried.

        :param attempt: Which attempt this is, for the report.
        :param episode: The episode to carry out.
        """
        try:
            episode.build_world()
        except Exception as failure:
            logger.info(
                "episode %d was not composed: %s: %s",
                attempt,
                type(failure).__name__,
                str(failure)[:120],
            )
            return
        logger.info("episode %d: %s", attempt, episode.task.instruction)
        try:
            episode.run()
        except Exception as failure:
            logger.info(
                "episode %d stopped with %s: %s: %s",
                attempt,
                episode.outcome,
                type(failure).__name__,
                str(failure)[:120],
                exc_info=failure,
            )
            self.count_what_the_scene_says(episode)
            return
        logger.info("episode %d finished: %s", attempt, episode.outcome)
        self.count_what_the_scene_says(episode)

    def count_what_the_scene_says(self, episode: TiagoRoboCasaTask) -> None:
        """
        Count the episode if the scene stands as the task asks, whatever became of the
        plan: a plan that stumbles once the object is where it belongs has still put it
        there, and the scene is what the task is about.

        :param episode: The episode that was carried out.
        """
        if episode.task_is_done:
            self.done += 1

    def report(self) -> None:
        """
        Say how the run went, and where its recording was written.
        """
        if self.tries_every_kitchen:
            logger.info(
                "%s: the task came off in %d of %d kitchens",
                self.task.task_name,
                self.done,
                self.attempts,
            )
        elif self.done == 0:
            logger.error(
                "%s: the task came off in none of the %d kitchens tried",
                self.task.task_name,
                self.attempts,
            )
        if self.settings.recording is not None:
            logger.info(
                "wrote a video of every kitchen to %s",
                self.settings.recording.path.parent,
            )


# %% asking for a run from the command line


def parser_of(description: str, tasks: list[str]) -> argparse.ArgumentParser:
    """
    :param description: What the run does.
    :param tasks: The names of the tasks that can be asked for.
    :return: The command line a run is asked for with.
    """
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("task", choices=tasks)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--record", nargs="?", const="", default=None, metavar="PATH")
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--noslip", type=int, default=10)
    parser.add_argument("--reach", type=float, default=None)
    parser.add_argument("--pitch", type=float, default=None, help="in degrees")
    parser.add_argument("--pyramidal", action="store_true")
    parser.add_argument("--left-arm", action="store_true")
    parser.add_argument("--counter-height", type=float, default=None)
    parser.add_argument("--category", default=None, help="what RoboCasa is to sample")
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--no-avoidance", action="store_true")
    parser.add_argument(
        "--vary-layout",
        action="store_true",
        help="compose each attempt in the next of RoboCasa's kitchen layouts",
    )
    parser.add_argument(
        "--first-layout",
        type=int,
        default=1,
        help="the layout the first attempt composes, with --vary-layout",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="try every kitchen and count the ones the task comes off in, rather than"
        " stopping at the first",
    )
    return parser


def settings_from(arguments: argparse.Namespace) -> EpisodeSettings:
    """
    :param arguments: What the command line was given.
    :return: How the episodes of that run are set up.
    """
    recording = None
    if arguments.record is not None:
        recording = SceneRecording(
            path=(
                Path(arguments.record)
                if arguments.record
                else Path.home() / "Downloads" / f"tiago_{arguments.task.lower()}.mp4"
            )
        )
    return EpisodeSettings(
        recording=recording,
        headless=not arguments.viewer,
        noslip_iterations=arguments.noslip,
        friction_cone=(
            mujoco.mjtCone.mjCONE_PYRAMIDAL
            if arguments.pyramidal
            else mujoco.mjtCone.mjCONE_ELLIPTIC
        ),
        works_with_the_left_arm=arguments.left_arm,
        collision_avoidance=not arguments.no_avoidance,
        seed=arguments.seed,
        reach=arguments.reach,
        grasp_pitch=None if arguments.pitch is None else math.radians(arguments.pitch),
        counter_height=arguments.counter_height,
        food_category=arguments.category,
        varies_the_layout=arguments.vary_layout,
        first_layout=arguments.first_layout,
    )

"""
A PR2 performs RoboCasa's blender tasks (``kitchen_blender.py``): it takes the blender's
lid off and lays it on the counter, puts it back on the jar, or presses the blender's
power button.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.pick_place import OntoCounter
from experiments.robocasa_atomic_tasks.robocasa_names import FixturePartSuffix
from experiments.robocasa_atomic_tasks.tasks import PressTask, RoboCasaAtomicTask
from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.fixtures import FixtureType

# %% the lid


@dataclass
class BlenderLidTask(RoboCasaAtomicTask):
    """
    A task about the blender's lid.
    """

    @staticmethod
    def lid(episode: RoboCasaEpisode) -> HasGraspCandidates:
        """
        :param episode: An episode of a blender task.
        :return: The blender's lid, as it may be grasped.
        """
        lid = episode.environment.blender.blender_lid
        return episode.loose_part(f"{lid.name}{FixturePartSuffix.MAIN_BODY}")


@dataclass
class OpenBlenderLid(BlenderLidTask):
    """
    Lift the lid off the blender's jar and lay it on the counter.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        lid = self.lid(episode)
        episode.pick_up(lid)
        counter = environment.get_fixture(FixtureType.COUNTER, ref=environment.blender)
        episode.place(
            lid,
            episode.placement.on_counter(
                counter, lid.root, near=OntoCounter.in_front_of_the_robot(episode)
            ),
        )


@dataclass
class CloseBlenderLid(BlenderLidTask):
    """
    Take the lid from the counter and set it on the blender's jar.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        lid = self.lid(episode)
        episode.pick_up(lid)
        episode.judge.mirror.synchronize()
        closed_position = np.asarray(
            environment.blender.get_lid_closed_pos(environment), dtype=float
        )
        origin = lid.root.global_pose.to_np()[:3, 3]
        underside_below_origin = origin[2] - episode.bounds.of_branch(lid.root).lower[2]
        episode.place(
            lid, [closed_position - np.array([0.0, 0.0, underside_below_origin])]
        )


# %% the power button


@dataclass
class TurnOnBlender(PressTask):
    """
    Press the blender's power button.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        blender = episode.environment.blender
        return f"{blender.name}{FixturePartSuffix.BLENDER_POWER_BUTTON}"

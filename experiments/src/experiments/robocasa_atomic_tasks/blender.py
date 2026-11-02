"""
A PR2 performs RoboCasa's blender tasks (``kitchen_blender.py``): it takes the
blender's lid off and lays it on the counter, puts it back on the jar, or presses the
blender's power button.

Run one with::

    python -m experiments.robocasa_atomic_tasks.blender TurnOnBlender
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.fixtures import FixtureType

from experiments.robocasa_atomic_tasks.episode import (
    PressTask,
    RoboCasaAtomicTask,
    RoboCasaEpisode,
    main,
)
from experiments.robocasa_atomic_tasks.pick_place import in_front_of_the_robot


def lid_body_name(episode: RoboCasaEpisode) -> str:
    """
    :param episode: An episode of a blender task.
    :return: The name RoboCasa gives the body of the blender's lid.
    """
    return f"{episode.environment.blender.blender_lid.name}_main"


@dataclass
class OpenBlenderLid(RoboCasaAtomicTask):
    """
    Lift the lid off the blender's jar and lay it on the counter.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        lid = episode.loose_part(lid_body_name(episode))
        episode.pick_up_graspable(lid)
        counter = environment.get_fixture(FixtureType.COUNTER, ref=environment.blender)
        episode.place(
            lid,
            episode.spots_on_counter(
                counter, lid.root, near=in_front_of_the_robot(episode)
            ),
        )


@dataclass
class CloseBlenderLid(RoboCasaAtomicTask):
    """
    Take the lid from the counter and set it on the blender's jar.
    """

    def solve(self, episode: RoboCasaEpisode) -> None:
        environment = episode.environment
        lid = episode.loose_part(lid_body_name(episode))
        episode.pick_up_graspable(lid)
        episode.mirror.synchronize()
        closed_position = np.asarray(
            environment.blender.get_lid_closed_pos(environment), dtype=float
        )
        lower, _ = episode.bounding_box(lid.root)
        origin = lid.root.global_pose.to_np()[:3, 3]
        underside_below_origin = origin[2] - lower[2]
        episode.place(
            lid,
            [closed_position - np.array([0.0, 0.0, underside_below_origin])],
        )


@dataclass
class TurnOnBlender(PressTask):
    """
    Press the blender's power button.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        return f"{episode.environment.blender.name}_power_button_main"


TASKS = [OpenBlenderLid, CloseBlenderLid, TurnOnBlender]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

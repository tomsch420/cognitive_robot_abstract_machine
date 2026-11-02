"""
A PR2 performs RoboCasa's stand mixer tasks (``kitchen_stand_mixer.py``): it tilts the
mixer's head up or down by taking hold of it.

Run one with::

    python -m experiments.robocasa_atomic_tasks.stand_mixer OpenStandMixerHead
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from experiments.robocasa_atomic_tasks.episode import (
    DoorShareTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class OpenStandMixerHead(DoorShareTask):
    """
    Tilt the stand mixer's head fully up.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return [episode.environment.stand_mixer._joint_names["head"]]


@dataclass
class CloseStandMixerHead(OpenStandMixerHead):
    """
    Tilt the raised head of the stand mixer back down.
    """

    share = 0.0


TASKS = [OpenStandMixerHead, CloseStandMixerHead]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

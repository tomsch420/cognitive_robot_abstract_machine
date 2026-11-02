"""
A PR2 performs RoboCasa's stand mixer tasks (``kitchen_stand_mixer.py``): it tilts the
mixer's head up or down by taking hold of it.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import FixtureJointRole
from experiments.robocasa_atomic_tasks.tasks import DoorShareTask

# %% the mixer's head


@dataclass
class OpenStandMixerHead(DoorShareTask):
    """
    Tilt the stand mixer's head fully up.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return [episode.environment.stand_mixer._joint_names[FixtureJointRole.HEAD]]


@dataclass
class CloseStandMixerHead(OpenStandMixerHead):
    """
    Tilt the raised head of the stand mixer back down.
    """

    share: float = 0.0

"""
A PR2 performs RoboCasa's electric kettle tasks (``kitchen_electric_kettle.py``): it
pushes the kettle's switch down, and swings its lid open or shut by hand.

RoboCasa's kettle opens its lid by itself once its lid button is pressed, but only as
the simulation's time passes, which a kinematic simulation does not advance; the PR2
opens the lid with its hand instead, which RoboCasa's check accepts just the same.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import FixtureJointRole
from experiments.robocasa_atomic_tasks.tasks import DoorShareTask, JointGoal, PushTask

# %% the switch


@dataclass
class TurnOnElectricKettle(PushTask):
    """
    Push the kettle's switch fully down.
    """

    def pushed_joint(self, episode: RoboCasaEpisode) -> JointGoal:
        name = episode.environment.electric_kettle._joint_names[FixtureJointRole.SWITCH]
        return JointGoal(name, episode.joints.world_position_at_share(name, 1.0))


# %% the lid


@dataclass
class CloseElectricKettleLid(DoorShareTask):
    """
    Swing the kettle's open lid shut.
    """

    share: float = 0.0

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return [episode.environment.electric_kettle._joint_names[FixtureJointRole.LID]]


@dataclass
class OpenElectricKettleLid(CloseElectricKettleLid):
    """
    Swing the kettle's lid fully open.
    """

    share: float = 1.0

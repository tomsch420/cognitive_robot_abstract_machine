"""
A PR2 performs RoboCasa's toaster oven tasks (``kitchen_toaster_oven.py``): it turns the
toaster oven's knobs and slides its rack or tray.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import FixtureJointRole
from experiments.robocasa_atomic_tasks.tasks import (
    DoorShareTask,
    JointGoal,
    JointGoalTask,
)

# %% knobs


@dataclass
class AdjustToasterOvenTemperature(JointGoalTask):
    """
    Turn the temperature knob up from a low setting, or down from a high one, as
    RoboCasa asks.
    """

    change: float = 0.3
    """
    How far the knob is turned, as a share of its range: RoboCasa asks for 0.15.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        environment = episode.environment
        name = environment.toaster_oven._joint_names[FixtureJointRole.TEMPERATURE]
        sign = 1.0 if environment.should_increase else -1.0
        share = float(np.clip(environment.initial_temp + sign * self.change, 0.0, 1.0))
        return [JointGoal(name, episode.joints.world_position_along_range(name, share))]


@dataclass
class TurnOnToasterOven(JointGoalTask):
    """
    Turn the timer knob of the toaster oven on.
    """

    timer_share: float = 0.3
    """
    How far the timer knob is turned, as a share of its range: RoboCasa counts the
    toaster oven on from 0.1.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> list[JointGoal]:
        name = episode.environment.toaster_oven._joint_names[FixtureJointRole.TIMER]
        return [
            JointGoal(
                name, episode.joints.world_position_along_range(name, self.timer_share)
            )
        ]


# %% racks


@dataclass
class SlideToasterOvenRack(DoorShareTask):
    """
    Pull the toaster oven's rack or tray fully out, or push it fully in, as RoboCasa
    asks.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        environment = episode.environment
        return [
            episode.joints.rack_joint_name(
                environment.toaster_oven, environment.rack_level
            )
        ]

    def target_share(self, episode: RoboCasaEpisode) -> float:
        return 1.0 if episode.environment.should_pull else 0.0

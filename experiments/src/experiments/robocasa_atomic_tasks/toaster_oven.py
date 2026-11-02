"""
A PR2 performs RoboCasa's toaster oven tasks (``kitchen_toaster_oven.py``): it turns the
toaster oven's knobs and slides its rack or tray.

Run one with::

    python -m experiments.robocasa_atomic_tasks.toaster_oven TurnOnToasterOven
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from typing_extensions import ClassVar, List

from experiments.robocasa_atomic_tasks.episode import (
    DoorShareTask,
    JointGoalTask,
    RoboCasaEpisode,
    main,
    rack_joint_name,
)


@dataclass
class AdjustToasterOvenTemperature(JointGoalTask):
    """
    Turn the temperature knob up from a low setting, or down from a high one, as
    RoboCasa asks.
    """

    change: ClassVar[float] = 0.3
    """
    How far the knob is turned, as a share of its range: RoboCasa asks for 0.15.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        environment = episode.environment
        name = environment.toaster_oven._joint_names["temperature"]
        sign = 1.0 if environment.should_increase else -1.0
        share = float(np.clip(environment.initial_temp + sign * self.change, 0.0, 1.0))
        return [(name, episode.world_position_along_range(name, share))]


@dataclass
class TurnOnToasterOven(JointGoalTask):
    """
    Turn the timer knob of the toaster oven on.
    """

    timer_share: ClassVar[float] = 0.3
    """
    How far the timer knob is turned, as a share of its range: RoboCasa counts the
    toaster oven on from 0.1.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        name = episode.environment.toaster_oven._joint_names["time"]
        return [(name, episode.world_position_along_range(name, self.timer_share))]


@dataclass
class SlideToasterOvenRack(DoorShareTask):
    """
    Pull the toaster oven's rack or tray fully out, or push it fully in, as RoboCasa
    asks.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        environment = episode.environment
        return [
            rack_joint_name(episode, environment.toaster_oven, environment.rack_level)
        ]

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        share = 1.0 if episode.environment.should_pull else 0.0
        return [
            (name, episode.world_position_at_share(name, share))
            for name in self.joint_names(episode)
        ]


TASKS = [AdjustToasterOvenTemperature, TurnOnToasterOven, SlideToasterOvenRack]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

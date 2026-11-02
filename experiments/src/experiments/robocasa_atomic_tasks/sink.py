"""
A PR2 performs RoboCasa's sink tasks (``kitchen_sink.py``): it takes hold of the
faucet's handle or spout and turns it.

Run one with::

    python -m experiments.robocasa_atomic_tasks.sink TurnOnSinkFaucet
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import ClassVar, List

from experiments.robocasa_atomic_tasks.episode import (
    JointGoalTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class TurnOnSinkFaucet(JointGoalTask):
    """
    Lift the faucet's handle until the water runs.
    """

    handle_position: ClassVar[float] = 0.47
    """
    Where the handle is lifted to, in radians: RoboCasa counts the water running above
    0.4 rad, and the handle stops at 0.5 rad.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        name = f"{episode.environment.sink.naming_prefix}handle_joint"
        return [(name, episode.world_position_of(name, self.handle_position))]


@dataclass
class TurnOffSinkFaucet(TurnOnSinkFaucet):
    """
    Push the faucet's handle down until the water stops.
    """

    handle_position: ClassVar[float] = 0.0


@dataclass
class TurnSinkSpout(JointGoalTask):
    """
    Swing the spout to the side RoboCasa asks for.
    """

    spout_position: ClassVar[float] = 0.8
    """
    How far the spout is swung, in radians: RoboCasa counts it turned to a side from
    pi / 6 on, and it stops at 1.05 rad either way.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        environment = episode.environment
        name = f"{environment.sink.naming_prefix}spout_joint"
        sign = 1.0 if environment.behavior == "right" else -1.0
        return [(name, episode.world_position_of(name, sign * self.spout_position))]


@dataclass
class AdjustWaterTemperature(JointGoalTask):
    """
    Swing the running faucet's handle towards hot or cold, as RoboCasa asks.
    """

    hot_share: ClassVar[float] = 0.8
    """
    How far along the temperature range the handle is swung for hot water; RoboCasa
    counts the water hot from 0.6 and cold up to 0.4.
    """

    def joint_goals(self, episode: RoboCasaEpisode) -> List[tuple[str, float]]:
        environment = episode.environment
        name = f"{environment.sink.naming_prefix}handle_temp_joint"
        share = (
            self.hot_share
            if environment.target_temp_state == "hot"
            else 1.0 - self.hot_share
        )
        return [(name, episode.world_position_along_range(name, share))]


TASKS = [TurnOnSinkFaucet, TurnOffSinkFaucet, TurnSinkSpout, AdjustWaterTemperature]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

"""
A PR2 performs RoboCasa's electric kettle tasks (``kitchen_electric_kettle.py``): it
pushes the kettle's switch down, and swings its lid open or shut by hand.

RoboCasa's kettle opens its lid by itself once its lid button is pressed, but only as
the simulation's time passes, which a kinematic simulation does not advance; the PR2
opens the lid with its hand instead, which RoboCasa's check accepts just the same.

Run one with::

    python -m experiments.robocasa_atomic_tasks.electric_kettle TurnOnElectricKettle
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from experiments.robocasa_atomic_tasks.episode import (
    DoorShareTask,
    PushTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class TurnOnElectricKettle(PushTask):
    """
    Push the kettle's switch fully down.
    """

    def pushed_joint(self, episode: RoboCasaEpisode) -> tuple[str, float]:
        name = episode.environment.electric_kettle._joint_names["switch"]
        return name, episode.world_position_at_share(name, 1.0)


@dataclass
class CloseElectricKettleLid(DoorShareTask):
    """
    Swing the kettle's open lid shut.
    """

    share = 0.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return [episode.environment.electric_kettle._joint_names["lid"]]


@dataclass
class OpenElectricKettleLid(CloseElectricKettleLid):
    """
    Swing the kettle's lid fully open.
    """

    share = 1.0


TASKS = [TurnOnElectricKettle, CloseElectricKettleLid, OpenElectricKettleLid]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

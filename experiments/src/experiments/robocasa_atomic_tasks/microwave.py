"""
A PR2 performs RoboCasa's microwave button tasks (``kitchen_microwave.py``): it presses
the microwave's start or stop button with its fingertips.

Run one with::

    python -m experiments.robocasa_atomic_tasks.microwave TurnOnMicrowave
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import ClassVar

from experiments.robocasa_atomic_tasks.episode import (
    PressTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class TurnOnMicrowave(PressTask):
    """
    Press the microwave's start button.
    """

    button: ClassVar[str] = "start_button"
    """
    The button pressed, as RoboCasa ends the name of its geom.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        return f"{episode.environment.microwave.name}_{self.button}"


@dataclass
class TurnOffMicrowave(TurnOnMicrowave):
    """
    Press the running microwave's stop button.
    """

    button: ClassVar[str] = "stop_button"


TASKS = [TurnOnMicrowave, TurnOffMicrowave]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

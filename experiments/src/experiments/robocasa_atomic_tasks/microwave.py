"""
A PR2 performs RoboCasa's microwave button tasks (``kitchen_microwave.py``): it presses
the microwave's start or stop button with its fingertips.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import MicrowaveButton
from experiments.robocasa_atomic_tasks.tasks import PressTask

# %% the microwave's buttons


@dataclass
class TurnOnMicrowave(PressTask):
    """
    Press the microwave's start button.
    """

    button: MicrowaveButton = MicrowaveButton.START
    """
    The button pressed.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        return f"{episode.environment.microwave.name}_{self.button}"


@dataclass
class TurnOffMicrowave(TurnOnMicrowave):
    """
    Press the running microwave's stop button.
    """

    button: MicrowaveButton = MicrowaveButton.STOP

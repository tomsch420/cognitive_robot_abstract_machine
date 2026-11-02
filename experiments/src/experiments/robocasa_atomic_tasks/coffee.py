"""
A PR2 performs RoboCasa's coffee machine tasks (``kitchen_coffee.py``): it puts a mug
under the coffee machine's spout, takes it from there onto the counter, or presses the
machine's start button.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.pick_place import OntoCounter
from experiments.robocasa_atomic_tasks.robocasa_names import FixturePartSuffix
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask, PressTask

# %% the mug


@dataclass
class CoffeeSetupMug(PickAndPlaceTask):
    """
    Take the mug from the counter and put it under the coffee machine's spout, carried
    in barely above the drip tray, since the machine's head overhangs it.
    """

    retreat_distance: float = 0.02

    def spots(self, episode: RoboCasaEpisode) -> list[np.ndarray]:
        environment = episode.environment
        machine = environment.coffee_machine
        episode.judge.mirror.synchronize()
        model, data = environment.sim.model, environment.sim.data
        site = model.site_name2id(
            f"{machine.naming_prefix}{FixturePartSuffix.COFFEE_RECEPTACLE_SITE}"
        )
        return [np.array(data.site_xpos[site], dtype=float)]


@dataclass
class CoffeeServeMug(OntoCounter):
    """
    Take the mug from under the coffee machine's spout and put it on the counter.
    """


# %% the start button


@dataclass
class StartCoffeeMachine(PressTask):
    """
    Press the coffee machine's start button.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        machine = episode.environment.coffee_machine
        return f"{machine.naming_prefix}{machine._start_button_names[0]}"

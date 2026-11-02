"""
A PR2 performs RoboCasa's coffee machine tasks (``kitchen_coffee.py``): it puts a mug
under the coffee machine's spout, takes it from there onto the counter, or presses the
machine's start button.

Run one with::

    python -m experiments.robocasa_atomic_tasks.coffee StartCoffeeMachine
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from typing_extensions import List

from experiments.robocasa_atomic_tasks.episode import (
    PickAndPlaceTask,
    PressTask,
    RoboCasaEpisode,
    main,
)
from experiments.robocasa_atomic_tasks.pick_place import OntoCounter


@dataclass
class CoffeeSetupMug(PickAndPlaceTask):
    """
    Take the mug from the counter and put it under the coffee machine's spout, carried
    in barely above the drip tray, since the machine's head overhangs it.
    """

    retreat_distance = 0.02

    def spots(self, episode: RoboCasaEpisode) -> List[np.ndarray]:
        environment = episode.environment
        machine = environment.coffee_machine
        episode.mirror.synchronize()
        model, data = environment.sim.model, environment.sim.data
        site = model.site_name2id(f"{machine.naming_prefix}receptacle_place_site")
        return [np.array(data.site_xpos[site], dtype=float)]


@dataclass
class CoffeeServeMug(OntoCounter):
    """
    Take the mug from under the coffee machine's spout and put it on the counter.
    """


@dataclass
class StartCoffeeMachine(PressTask):
    """
    Press the coffee machine's start button.
    """

    def button_geom(self, episode: RoboCasaEpisode) -> str:
        machine = episode.environment.coffee_machine
        return f"{machine.naming_prefix}{machine._start_button_names[0]}"


TASKS = [CoffeeSetupMug, CoffeeServeMug, StartCoffeeMachine]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

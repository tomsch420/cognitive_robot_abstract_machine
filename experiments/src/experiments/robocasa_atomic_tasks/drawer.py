"""
A PR2 performs RoboCasa's drawer tasks (``kitchen_drawer.py``): it pulls drawers and
racks out, or pushes them in, by their handles or front edges.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import (
    FixtureJointRole,
    FridgeCompartment,
)
from experiments.robocasa_atomic_tasks.tasks import DoorShareTask

# %% drawers


@dataclass
class OpenDrawer(DoorShareTask):
    """
    Pull the drawer RoboCasa picked fully open.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return list(episode.environment.drawer.door_joint_names)


@dataclass
class CloseDrawer(OpenDrawer):
    """
    Push the open drawer RoboCasa picked shut.
    """

    share: float = 0.0


@dataclass
class OpenFridgeDrawer(DoorShareTask):
    """
    Pull the fridge's drawer RoboCasa checks fully open.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return episode.environment.fridge._get_drawer_joints(
            compartment=FridgeCompartment.FRIDGE, drawer_rack_index=-1
        )


@dataclass
class CloseFridgeDrawer(OpenFridgeDrawer):
    """
    Push the fridge's open drawer shut.
    """

    share: float = 0.0


# %% racks


@dataclass
class SlideDishwasherRack(DoorShareTask):
    """
    Pull the dishwasher's rack fully out, or push it fully in, as RoboCasa asks.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return [episode.environment.dishwasher._joint_names[FixtureJointRole.RACK]]

    def target_share(self, episode: RoboCasaEpisode) -> float:
        return 1.0 if episode.environment.should_pull else 0.0

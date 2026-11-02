"""
A PR2 performs RoboCasa's drawer tasks (``kitchen_drawer.py``): it pulls drawers and
racks out, or pushes them in, by their handles or front edges.

Run one with::

    python -m experiments.robocasa_atomic_tasks.drawer OpenDrawer
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from experiments.robocasa_atomic_tasks.episode import (
    DoorShareTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class OpenDrawer(DoorShareTask):
    """
    Pull the drawer RoboCasa picked fully open.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return list(episode.environment.drawer.door_joint_names)


@dataclass
class CloseDrawer(DoorShareTask):
    """
    Push the open drawer RoboCasa picked shut.
    """

    share = 0.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return list(episode.environment.drawer.door_joint_names)


@dataclass
class SlideDishwasherRack(DoorShareTask):
    """
    Pull the dishwasher's rack fully out, or push it fully in, as RoboCasa asks.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return [episode.environment.dishwasher._joint_names["rack"]]

    def joint_goals(self, episode: RoboCasaEpisode):
        share = 1.0 if episode.environment.should_pull else 0.0
        return [
            (name, episode.world_position_at_share(name, share))
            for name in self.joint_names(episode)
        ]


@dataclass
class OpenFridgeDrawer(DoorShareTask):
    """
    Pull the fridge's drawer RoboCasa checks fully open.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return episode.environment.fridge._get_drawer_joints(
            compartment="fridge", drawer_rack_index=-1
        )


@dataclass
class CloseFridgeDrawer(DoorShareTask):
    """
    Push the fridge's open drawer shut.
    """

    share = 0.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return episode.environment.fridge._get_drawer_joints(
            compartment="fridge", drawer_rack_index=-1
        )


TASKS = [OpenDrawer, CloseDrawer, SlideDishwasherRack, OpenFridgeDrawer, CloseFridgeDrawer]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

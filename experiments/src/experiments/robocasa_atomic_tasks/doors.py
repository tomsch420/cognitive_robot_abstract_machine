"""
A PR2 performs RoboCasa's door tasks (``kitchen_doors.py``): it swings the doors of
cabinets, the microwave, the fridge, the oven, the dishwasher and the toaster oven open
or shut by their handles.

Run one with::

    python -m experiments.robocasa_atomic_tasks.doors OpenCabinet
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

from experiments.robocasa_atomic_tasks.episode import (
    DoorShareTask,
    RoboCasaEpisode,
    main,
)


@dataclass
class ManipulateFixtureDoors(DoorShareTask):
    """
    Swing every door of the fixture RoboCasa refers to as ``fxtr`` to a share of its
    travel.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return list(episode.environment.fxtr.door_joint_names)


@dataclass
class OpenCabinet(ManipulateFixtureDoors):
    """
    Swing the doors of the cabinet open.
    """

    share = 1.0


@dataclass
class CloseCabinet(ManipulateFixtureDoors):
    """
    Swing the open doors of the cabinet shut.
    """

    share = 0.0


@dataclass
class OpenMicrowave(ManipulateFixtureDoors):
    """
    Swing the microwave's door open.
    """

    share = 1.0


@dataclass
class CloseMicrowave(ManipulateFixtureDoors):
    """
    Swing the microwave's open door shut.
    """

    share = 0.0


@dataclass
class OpenFridge(ManipulateFixtureDoors):
    """
    Swing the doors of the fridge's cooling compartment open; RoboCasa leaves a freezer
    below it out of the task.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return list(episode.environment.fxtr._fridge_door_joint_names)


@dataclass
class CloseFridge(OpenFridge):
    """
    Swing the open doors of the fridge's cooling compartment shut.
    """

    share = 0.0


@dataclass
class OpenOven(ManipulateFixtureDoors):
    """
    Let the oven's door down; in layout 2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    share = 1.0


@dataclass
class CloseOven(ManipulateFixtureDoors):
    """
    Swing the oven's let down door shut; in layout 2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    share = 0.0


@dataclass
class OpenDishwasher(ManipulateFixtureDoors):
    """
    Let the dishwasher's door down.
    """

    share = 1.0


@dataclass
class CloseDishwasher(ManipulateFixtureDoors):
    """
    Swing the dishwasher's let down door shut.
    """

    share = 0.0


@dataclass
class OpenToasterOvenDoor(DoorShareTask):
    """
    Let the toaster oven's door down.
    """

    share = 1.0

    def joint_names(self, episode: RoboCasaEpisode) -> List[str]:
        return [episode.environment.toaster_oven._joint_names["door"]]


@dataclass
class CloseToasterOvenDoor(OpenToasterOvenDoor):
    """
    Swing the toaster oven's let down door shut.
    """

    share = 0.0


TASKS = [
    OpenCabinet,
    CloseCabinet,
    OpenMicrowave,
    CloseMicrowave,
    OpenFridge,
    CloseFridge,
    OpenOven,
    CloseOven,
    OpenDishwasher,
    CloseDishwasher,
    OpenToasterOvenDoor,
    CloseToasterOvenDoor,
]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

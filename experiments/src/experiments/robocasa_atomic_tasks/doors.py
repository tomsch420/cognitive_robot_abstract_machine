"""
A PR2 performs RoboCasa's door tasks (``kitchen_doors.py``): it swings the doors of
cabinets, the microwave, the fridge, the oven, the dishwasher and the toaster oven open
or shut by their handles.
"""

from __future__ import annotations

from dataclasses import dataclass

from experiments.robocasa_atomic_tasks.door_handles import RoboCasaDoorHandleSplitter
from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.let_down_doors import (
    LetTheDoorDownByItsHandleAction,
)
from experiments.robocasa_atomic_tasks.robocasa_names import FixtureJointRole
from experiments.robocasa_atomic_tasks.tasks import DoorShareTask
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Door

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

# %% the doors of the fixture a task refers to


@dataclass
class ManipulateFixtureDoors(DoorShareTask):
    """
    Swing every door of the fixture RoboCasa refers to as ``fxtr`` to a share of its
    travel.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return list(episode.environment.fxtr.door_joint_names)

    def doors(self, episode: RoboCasaEpisode) -> list[Door]:
        """
        :param episode: The episode the task is performed in.
        :return: The doors hanging from the joints of :meth:`joint_names`.
        """
        joint_names = self.joint_names(episode)
        return [
            door
            for door in episode.world.get_semantic_annotations_by_type(Door)
            if door.root.parent_connection.name.name in joint_names
        ]


@dataclass
class OpenCabinet(ManipulateFixtureDoors):
    """
    Swing the doors of the cabinet open.
    """


@dataclass
class CloseCabinet(ManipulateFixtureDoors):
    """
    Swing the open doors of the cabinet shut.
    """

    share: float = 0.0


@dataclass
class OpenMicrowave(ManipulateFixtureDoors):
    """
    Swing the microwave's door open.
    """


@dataclass
class CloseMicrowave(ManipulateFixtureDoors):
    """
    Swing the microwave's open door shut.
    """

    share: float = 0.0


@dataclass
class OpenFridge(ManipulateFixtureDoors):
    """
    Swing the doors of the fridge's cooling compartment open; RoboCasa leaves a freezer
    below it out of the task.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return list(episode.environment.fxtr._fridge_door_joint_names)


@dataclass
class CloseFridge(OpenFridge):
    """
    Swing the open doors of the fridge's cooling compartment shut.
    """

    share: float = 0.0


@dataclass
class OpenOven(ManipulateFixtureDoors):
    """
    Let the oven's door down; in layout 2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002

    def compose(self) -> RoboCasaEpisode:
        """
        Differs from :meth:`RoboCasaAtomicTask.compose` in splitting the handle off the
        oven's door into a body of its own when the episode is simulated physically.
        """
        episode = super().compose()
        if not self.physics:
            return episode
        for door in self.doors(episode):
            RoboCasaDoorHandleSplitter().split_handle_off(door=door)
        return episode

    def solve(self, episode: RoboCasaEpisode) -> None:
        """
        Differs from :meth:`DoorShareTask.solve` in letting the door down by
        :class:`LetTheDoorDownByItsHandleAction` when the episode is simulated
        physically.
        """
        if not episode.physically_simulated:
            super().solve(episode)
            return
        for door in self.doors(episode):
            episode.run(
                [
                    LetTheDoorDownByItsHandleAction.from_handle(
                        handle=door.handle, arm=episode.arm
                    )
                ]
            ).raise_if_failed()


@dataclass
class CloseOven(ManipulateFixtureDoors):
    """
    Swing the oven's let down door shut; in layout 2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    share: float = 0.0


@dataclass
class OpenDishwasher(ManipulateFixtureDoors):
    """
    Let the dishwasher's door down.
    """


@dataclass
class CloseDishwasher(ManipulateFixtureDoors):
    """
    Swing the dishwasher's let down door shut.
    """

    share: float = 0.0


# %% the toaster oven's door


@dataclass
class OpenToasterOvenDoor(DoorShareTask):
    """
    Let the toaster oven's door down.
    """

    def joint_names(self, episode: RoboCasaEpisode) -> list[str]:
        return [episode.environment.toaster_oven._joint_names[FixtureJointRole.DOOR]]


@dataclass
class CloseToasterOvenDoor(OpenToasterOvenDoor):
    """
    Swing the toaster oven's let down door shut.
    """

    share: float = 0.0

"""
A PR2 performs RoboCasa's pick and place tasks (``kitchen_pick_place.py``): it picks the
object RoboCasa names up and puts it down inside a fixture, on a counter, or on or in
another object.

Where the object may go is read off RoboCasa's own description of the kitchen: the
inside of a fixture as RoboCasa bounds it, the regions of a counter's top RoboCasa
places objects in, and the top of a receptacle.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
from typing_extensions import TYPE_CHECKING, Sequence

from experiments.robocasa_atomic_tasks.episode import RoboCasaEpisode
from experiments.robocasa_atomic_tasks.robocasa_names import (
    FixtureRegionPrefix,
    RoboCasaObjectName,
)
from experiments.robocasa_atomic_tasks.tasks import PickAndPlaceTask
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

if TYPE_CHECKING:
    from robocasa.models.fixtures import Fixture

# %% where an object goes


@dataclass
class IntoFixture(PickAndPlaceTask, ABC):
    """
    Put the object down inside a fixture.
    """

    margin: float = 0.02
    """
    How far the object stays from the sides of the inside, in metres.
    """

    dropped_in: bool = False
    """
    Whether the object is let go of at the top of the inside, to drop to its floor,
    rather than set down on the floor.
    """

    @abstractmethod
    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        """
        :param episode: The episode the task is performed in.
        :return: The fixture the object is put into.
        """

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        """
        :param episode: The episode the task is performed in.
        :return: Which of the fixture's inside regions the object may go to; all if
            ``None``.
        """
        return None

    def spots(self, episode: RoboCasaEpisode) -> list[np.ndarray]:
        return episode.placement.inside(
            self.fixture(episode),
            episode.object_body(self.picked_object),
            region_names=self.regions(episode),
            margin=self.margin,
            dropped_in=self.dropped_in,
        )


@dataclass
class OntoCounter(PickAndPlaceTask):
    """
    Put the object down on the counter RoboCasa refers to, near the robot.
    """

    def spots(self, episode: RoboCasaEpisode) -> list[np.ndarray]:
        return episode.placement.on_counter(
            episode.environment.counter,
            episode.object_body(self.picked_object),
            near=self.in_front_of_the_robot(episode),
        )

    @staticmethod
    def in_front_of_the_robot(
        episode: RoboCasaEpisode, reach: float = 0.6
    ) -> np.ndarray:
        """
        :param episode: An episode.
        :param reach: How far in front of the robot's base the point lies, in metres.
        :return: The point on the floor that far in front of the robot.
        """
        world_T_base = episode.robot.root.global_pose.to_np()
        return world_T_base[:3, 3] + reach * world_T_base[:3, 0]


@dataclass
class OntoReceptacle(PickAndPlaceTask):
    """
    Put the object down on or in another of the task's objects.
    """

    receptacle: RoboCasaObjectName = RoboCasaObjectName.CONTAINER
    """
    The object the moved object goes on or in.
    """

    def spots(self, episode: RoboCasaEpisode) -> list[np.ndarray]:
        return episode.placement.on_receptacle(episode.object_body(self.receptacle))


# %% RoboCasa's pick and place tasks


@dataclass
class PickPlaceCounterToCabinet(IntoFixture):
    """
    Take the object from the counter and put it inside the open cabinet.
    """

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.cab


@dataclass
class PickPlaceCabinetToCounter(OntoCounter):
    """
    Take the object out of the open cabinet and put it on the counter.
    """


@dataclass
class PickPlaceCounterToSink(IntoFixture):
    """
    Take the object from the counter and drop it into the sink from just above its rim.
    """

    dropped_in: bool = True

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.sink


@dataclass
class PickPlaceSinkToCounter(OntoReceptacle):
    """
    Take the object out of the sink and put it on the plate on the counter.
    """


@dataclass
class PickPlaceCounterToMicrowave(OntoReceptacle):
    """
    Take the object from the counter and put it on the plate in the open microwave.
    """


@dataclass
class PickPlaceMicrowaveToCounter(OntoReceptacle):
    """
    Take the object from the plate in the open microwave and put it on the plate on the
    counter.
    """


@dataclass
class PickPlaceCounterToOven(OntoReceptacle):
    """
    Take the object from the counter and put it on the tray in the open oven; in layout
    2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    receptacle: RoboCasaObjectName = RoboCasaObjectName.OVEN_TRAY


@dataclass
class PickPlaceCounterToStove(OntoReceptacle):
    """
    Take the food from the plate on the counter and put it in the pan on the stove.
    """


@dataclass
class PickPlaceStoveToCounter(OntoReceptacle):
    """
    Take the food out of the pan on the stove and put it on the plate on the counter.
    """


@dataclass
class PickPlaceToasterToCounter(OntoReceptacle):
    """
    Take the toasted item out of the toaster and put it on the plate.
    """

    receptacle: RoboCasaObjectName = RoboCasaObjectName.PLATE


@dataclass
class PickPlaceCounterToToasterOven(IntoFixture):
    """
    Take the object from the counter and put it on the rack or tray of the toaster oven
    RoboCasa slid out.
    """

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.toaster_oven

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        return [episode.environment.chosen_toaster_receptacle]


@dataclass
class PickPlaceToasterOvenToCounter(OntoReceptacle):
    """
    Take the object from the toaster oven's tray and put it on the plate on the counter.
    """


@dataclass
class PickPlaceCounterToStandMixer(IntoFixture):
    """
    Take the object from the counter and put it into the stand mixer's bowl.
    """

    margin: float = 0.0

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.stand_mixer

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        return [FixtureRegionPrefix.STAND_MIXER_BOWL]


@dataclass
class PickPlaceFridgeShelfToDrawer(IntoFixture):
    """
    Take the object from a shelf of the fridge and put it into the fridge's top drawer,
    which RoboCasa opened.
    """

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.fridge

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        drawers = sorted(
            name
            for name in episode.environment.fridge.get_int_sites(relative=False)
            if name.startswith(FixtureRegionPrefix.FRIDGE_DRAWER)
        )
        return drawers[-1:]


@dataclass
class PickPlaceFridgeDrawerToShelf(IntoFixture):
    """
    Take the object out of the fridge's open drawer and put it on a shelf of the fridge.
    """

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.fridge

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        return [
            name
            for name in episode.environment.fridge.get_int_sites(relative=False)
            if name.startswith(FixtureRegionPrefix.FRIDGE_SHELF)
        ]


@dataclass
class PickPlaceCounterToDrawer(IntoFixture):
    """
    Take the object from the counter and drop it into the open drawer from just above
    it.
    """

    dropped_in: bool = True

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.drawer


@dataclass
class PickPlaceDrawerToCounter(OntoCounter):
    """
    Take the object out of the open drawer and put it on the counter.
    """


@dataclass
class PickPlaceCounterToBlender(IntoFixture):
    """
    Take the object from the counter and drop it into the blender's jar.
    """

    margin: float = 0.0

    def fixture(self, episode: RoboCasaEpisode) -> Fixture:
        return episode.environment.blender


@dataclass
class CheesyBread(OntoReceptacle):
    """
    Take the wedge of cheese and put it on the slice of bread on the cutting board.
    """

    picked_object: RoboCasaObjectName = RoboCasaObjectName.CHEESE
    receptacle: RoboCasaObjectName = RoboCasaObjectName.BREAD


@dataclass
class MakeIcedCoffee(OntoReceptacle):
    """
    Take an ice cube from the bowl and drop it into the glass of coffee.
    """

    picked_object: RoboCasaObjectName = RoboCasaObjectName.ICE_CUBE
    receptacle: RoboCasaObjectName = RoboCasaObjectName.CUP


@dataclass
class PackDessert(OntoReceptacle):
    """
    Take the dessert and put it into the container holding the cooked food.
    """

    picked_object: RoboCasaObjectName = RoboCasaObjectName.DESSERT
    receptacle: RoboCasaObjectName = RoboCasaObjectName.COOKED_FOOD_CONTAINER

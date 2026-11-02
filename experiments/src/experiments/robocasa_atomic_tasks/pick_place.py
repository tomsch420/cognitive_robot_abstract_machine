"""
A PR2 performs RoboCasa's pick and place tasks (``kitchen_pick_place.py``): it picks the
object RoboCasa names up and puts it down inside a fixture, on a counter, or on or in
another object.

Where the object may go is read off RoboCasa's own description of the kitchen: the
inside of a fixture as RoboCasa bounds it, the regions of a counter's top RoboCasa
places objects in, and the top of a receptacle. A kinematic simulation has no gravity,
so the object is let go of just above its spot and lowered until it touches what it
lands on.

Run one with::

    python -m experiments.robocasa_atomic_tasks.pick_place PickPlaceCounterToCabinet
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from typing_extensions import ClassVar, List, Sequence

from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType

from experiments.robocasa_atomic_tasks.episode import (
    PickAndPlaceTask,
    RoboCasaEpisode,
    main,
)


def in_front_of_the_robot(episode: RoboCasaEpisode, reach: float = 0.6) -> np.ndarray:
    """
    :param episode: An episode.
    :param reach: How far in front of the robot's base the point lies, in metres.
    :return: The point on the floor that far in front of the robot.
    """
    world_T_base = episode.robot.root.global_pose.to_np()
    return world_T_base[:3, 3] + reach * world_T_base[:3, 0]


@dataclass
class IntoFixture(PickAndPlaceTask):
    """
    Put the object down inside a fixture.
    """

    fixture_attribute: ClassVar[str] = ""
    """
    The attribute of RoboCasa's task the fixture is held in.
    """

    region_names: ClassVar[Sequence[str] | None] = None
    """
    Which of the fixture's inside regions the object may go to; all if not given.
    """

    margin: ClassVar[float] = 0.02
    """
    How far the object stays from the sides of the inside, in metres.
    """

    dropped_in: ClassVar[bool] = False
    """
    Whether the object is let go of at the top of the inside, to drop to its floor,
    rather than set down on the floor.
    """

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        """
        :param episode: The episode the task is performed in.
        :return: Which of the fixture's inside regions the object may go to.
        """
        return self.region_names

    def spots(self, episode: RoboCasaEpisode) -> List[np.ndarray]:
        body = episode.object_body(self.picked_object)
        return episode.spots_inside(
            getattr(episode.environment, self.fixture_attribute),
            self.regions(episode),
            body=body,
            margin=self.margin,
            dropped_in=self.dropped_in,
        )


@dataclass
class OntoCounter(PickAndPlaceTask):
    """
    Put the object down on the counter RoboCasa refers to, near the robot.
    """

    def spots(self, episode: RoboCasaEpisode) -> List[np.ndarray]:
        return episode.spots_on_counter(
            episode.environment.counter,
            episode.object_body(self.picked_object),
            near=in_front_of_the_robot(episode),
        )


@dataclass
class OntoReceptacle(PickAndPlaceTask):
    """
    Put the object down on or in another of the task's objects.
    """

    receptacle: ClassVar[str] = "container"
    """
    The name RoboCasa gives the object the moved object goes on or in.
    """

    def spots(self, episode: RoboCasaEpisode) -> List[np.ndarray]:
        return episode.spots_on_receptacle(
            episode.object_body(self.receptacle),
            episode.object_body(self.picked_object),
        )


@dataclass
class PickPlaceCounterToCabinet(IntoFixture):
    """
    Take the object from the counter and put it inside the open cabinet.
    """

    fixture_attribute = "cab"


@dataclass
class PickPlaceCabinetToCounter(OntoCounter):
    """
    Take the object out of the open cabinet and put it on the counter.
    """


@dataclass
class PickPlaceCounterToSink(IntoFixture):
    """
    Take the object from the counter and drop it into the sink from just above its
    rim, as reaching down to the basin's floor runs the forearm into the counter.
    """

    fixture_attribute = "sink"
    dropped_in = True


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
    Take the object from the plate in the open microwave and put it on the plate on
    the counter.
    """


@dataclass
class PickPlaceCounterToOven(OntoReceptacle):
    """
    Take the object from the counter and put it on the tray in the open oven; in
    layout 2, since layout 1 has no oven.
    """

    layout: LayoutType = LayoutType.LAYOUT002
    receptacle = "oven_tray"


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

    receptacle = "plate"


@dataclass
class PickPlaceCounterToToasterOven(IntoFixture):
    """
    Take the object from the counter and put it on the rack or tray of the toaster
    oven RoboCasa slid out.
    """

    fixture_attribute = "toaster_oven"

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        return [episode.environment.chosen_toaster_receptacle]


@dataclass
class PickPlaceToasterOvenToCounter(OntoReceptacle):
    """
    Take the object from the toaster oven's tray and put it on the plate on the
    counter.
    """


@dataclass
class PickPlaceCounterToStandMixer(IntoFixture):
    """
    Take the object from the counter and put it into the stand mixer's bowl.
    """

    fixture_attribute = "stand_mixer"
    region_names = ("bowl",)
    margin = 0.0


@dataclass
class PickPlaceFridgeShelfToDrawer(IntoFixture):
    """
    Take the object from a shelf of the fridge and put it into the fridge's top
    drawer, which RoboCasa opened.
    """

    fixture_attribute = "fridge"

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        drawers = sorted(
            name
            for name in episode.environment.fridge.get_int_sites(relative=False)
            if name.startswith("fridge_drawer")
        )
        return drawers[-1:]


@dataclass
class PickPlaceFridgeDrawerToShelf(IntoFixture):
    """
    Take the object out of the fridge's open drawer and put it on a shelf of the
    fridge.
    """

    fixture_attribute = "fridge"

    def regions(self, episode: RoboCasaEpisode) -> Sequence[str] | None:
        return [
            name
            for name in episode.environment.fridge.get_int_sites(relative=False)
            if name.startswith("fridge_shelf")
        ]


@dataclass
class PickPlaceCounterToDrawer(IntoFixture):
    """
    Take the object from the counter and drop it into the open drawer from just above
    it, as reaching down to the drawer's floor runs the forearm into the counter above.
    """

    fixture_attribute = "drawer"
    dropped_in = True


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

    fixture_attribute = "blender"
    margin = 0.0


@dataclass
class CheesyBread(OntoReceptacle):
    """
    Take the wedge of cheese and put it on the slice of bread on the cutting board.
    """

    picked_object = "cheese"
    receptacle = "bread"


@dataclass
class MakeIcedCoffee(OntoReceptacle):
    """
    Take an ice cube from the bowl and drop it into the glass of coffee.
    """

    picked_object = "ice_cube1"
    receptacle = "cup"


@dataclass
class PackDessert(OntoReceptacle):
    """
    Take the dessert and put it into the container holding the cooked food.
    """

    picked_object = "dessert"
    receptacle = "cooked_food_container"


TASKS = [
    PickPlaceCounterToCabinet,
    PickPlaceCabinetToCounter,
    PickPlaceCounterToSink,
    PickPlaceSinkToCounter,
    PickPlaceCounterToMicrowave,
    PickPlaceMicrowaveToCounter,
    PickPlaceCounterToOven,
    PickPlaceCounterToStove,
    PickPlaceStoveToCounter,
    PickPlaceToasterToCounter,
    PickPlaceCounterToToasterOven,
    PickPlaceToasterOvenToCounter,
    PickPlaceCounterToStandMixer,
    PickPlaceFridgeShelfToDrawer,
    PickPlaceFridgeDrawerToShelf,
    PickPlaceCounterToDrawer,
    PickPlaceDrawerToCounter,
    PickPlaceCounterToBlender,
    CheesyBread,
    MakeIcedCoffee,
    PackDessert,
]
"""
Every task of this module.
"""

if __name__ == "__main__":
    main(__spec__.name)

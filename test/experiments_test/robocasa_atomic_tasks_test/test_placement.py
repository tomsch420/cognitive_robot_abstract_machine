"""
Where an object may be put down.
"""

from __future__ import annotations

import numpy as np

from experiments.robocasa_atomic_tasks.fixtures import KitchenFixtures
from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.placement import FixtureRegion, PlacementSpots

from .conftest import EmptyRoboCasaKitchen

# %% fixtures


def placement_in(kitchen_of_boxes) -> PlacementSpots:
    """
    :return: The placement spots in a kitchen RoboCasa composed nothing into.
    """
    world = kitchen_of_boxes.world
    environment = EmptyRoboCasaKitchen()
    bounds = BodyBounds(world)
    return PlacementSpots(
        environment=environment,
        world=world,
        bounds=bounds,
        fixtures=KitchenFixtures(environment=environment, world=world, bounds=bounds),
    )


def region_of(width: float, depth: float, height: float) -> FixtureRegion:
    """
    :return: A region at the origin, spanning the given extents along the axes.
    """
    return FixtureRegion.from_corners(
        [
            np.zeros(3),
            np.array([width, 0.0, 0.0]),
            np.array([0.0, depth, 0.0]),
            np.array([0.0, 0.0, height]),
        ]
    )


# %% regions


def test_a_region_reaches_as_high_as_its_corner_above():
    assert region_of(1.0, 1.0, 0.4).height == 0.4


def test_an_object_fits_in_a_region_on_spots_on_its_floor_middle_of_the_front_first(
    kitchen_of_boxes,
):
    spots = placement_in(kitchen_of_boxes).in_region(
        region_of(1.0, 1.0, 0.4), kitchen_of_boxes.plate
    )

    assert spots
    assert all(spot[2] == 0.0 for spot in spots)
    distances = [np.linalg.norm(spot[:2] - [0.5, 0.0]) for spot in spots]
    assert distances == sorted(distances)


def test_an_object_dropped_in_is_let_go_of_at_the_top_of_the_region(kitchen_of_boxes):
    height = 0.4

    spots = placement_in(kitchen_of_boxes).in_region(
        region_of(1.0, 1.0, height), kitchen_of_boxes.plate, dropped_in=True
    )

    assert spots
    assert all(spot[2] == height for spot in spots)


def test_no_spot_is_found_in_a_region_narrower_than_the_object(kitchen_of_boxes):
    narrower = kitchen_of_boxes.plate_scale.x / 2

    assert (
        placement_in(kitchen_of_boxes).in_region(
            region_of(narrower, 1.0, 0.4), kitchen_of_boxes.plate
        )
        == []
    )


# %% receptacles


def test_an_object_goes_on_a_receptacle_above_its_middle_first(kitchen_of_boxes):
    number_around_the_middle = 6

    spots = placement_in(kitchen_of_boxes).on_receptacle(
        kitchen_of_boxes.plate, number_around_the_middle=number_around_the_middle
    )

    top = kitchen_of_boxes.plate_center[2] + kitchen_of_boxes.plate_scale.z / 2
    np.testing.assert_allclose(
        spots[0], np.append(kitchen_of_boxes.plate_center[:2], top)
    )
    assert len(spots) == 1 + number_around_the_middle


# %% the object put down


def test_an_object_is_put_down_with_the_middle_of_its_underside_above_its_spot(
    kitchen_of_boxes,
):
    spot = np.array([2.0, 2.0, 0.0])
    clearance = 0.01

    pose = placement_in(kitchen_of_boxes).pose_with_underside_at(
        kitchen_of_boxes.plate, spot, clearance=clearance
    )

    expected = spot + [0.0, 0.0, kitchen_of_boxes.plate_scale.z / 2 + clearance]
    np.testing.assert_allclose(pose.to_np()[:3, 3], expected, atol=1e-6)

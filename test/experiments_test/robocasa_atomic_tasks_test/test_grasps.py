"""
The grasps the PR2 tries on the kitchen's parts and objects.
"""

from __future__ import annotations

import numpy as np

from experiments.robocasa_atomic_tasks.geometry import BodyBounds
from experiments.robocasa_atomic_tasks.grasps import KitchenGrasps
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle

# %% fixtures


def grasps_facing_along_x(kitchen_of_boxes) -> KitchenGrasps:
    """
    :return: The grasps of a robot facing the kitchen along the world's x-axis.
    """
    return KitchenGrasps(
        bounds=BodyBounds(kitchen_of_boxes.world),
        kitchen_direction=np.array([1.0, 0.0, 0.0]),
    )


def approach_of(grasp) -> np.ndarray:
    """
    :return: The direction a grasp comes in along, in the world's root frame.
    """
    return grasp.world_T_grasp.to_np()[:3, 0]


# %% grasps on a part


def test_a_part_is_grasped_from_every_side_from_above_and_rising_from_below(
    kitchen_of_boxes,
):
    grasps = grasps_facing_along_x(kitchen_of_boxes).on_part(kitchen_of_boxes.handle)

    horizontal_and_from_above = 5
    closings_each = 2
    rising = 4
    assert len(grasps) == horizontal_and_from_above * closings_each + rising


def test_a_part_grasped_level_closes_the_fingers_across_its_thinner_side_first(
    kitchen_of_boxes,
):
    grasps = grasps_facing_along_x(kitchen_of_boxes).on_part(kitchen_of_boxes.handle)

    first = grasps[0].world_T_grasp.to_np()
    np.testing.assert_allclose(first[:3, 0], [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(np.abs(first[:3, 1]), [0.0, 1.0, 0.0], atol=1e-9)


def test_a_grasp_on_a_part_lies_no_deeper_than_its_depth_behind_its_face(
    kitchen_of_boxes,
):
    depth = 0.005
    grasps = grasps_facing_along_x(kitchen_of_boxes).on_part(
        kitchen_of_boxes.handle, depth=depth
    )

    front_face = kitchen_of_boxes.handle_center[0] - kitchen_of_boxes.handle_scale.x / 2
    assert np.isclose(grasps[0].world_T_grasp.to_np()[0, 3], front_face + depth)


# %% grasps from above


def test_an_object_is_taken_from_above_once_per_pitch_and_horizontal_extent(
    kitchen_of_boxes,
):
    plate = Handle(root=kitchen_of_boxes.plate)
    pitches = (np.pi / 4, np.pi / 3)

    grasps = grasps_facing_along_x(kitchen_of_boxes).from_above(plate, pitches=pitches)

    assert len(grasps) == 2 * len(pitches)
    assert all(approach_of(grasp)[2] < 0 for grasp in grasps)


# %% grasps along the kitchen


def test_only_grasps_coming_in_along_the_kitchen_or_from_above_are_kept(
    kitchen_of_boxes,
):
    kitchen_grasps = grasps_facing_along_x(kitchen_of_boxes)
    grasps = kitchen_grasps.on_part(kitchen_of_boxes.handle)

    kept = kitchen_grasps.approaching_along_the_kitchen(grasps)

    assert kept
    for grasp in kept:
        approach = approach_of(grasp)
        assert approach[0] > 0 or approach[2] <= -0.5


def test_level_grasps_come_before_rising_ones_and_those_from_above_last(
    kitchen_of_boxes,
):
    kitchen_grasps = grasps_facing_along_x(kitchen_of_boxes)

    kept = kitchen_grasps.approaching_along_the_kitchen(
        kitchen_grasps.on_part(kitchen_of_boxes.handle)
    )

    rises = [approach_of(grasp)[2] for grasp in kept]
    assert np.isclose(rises[0], 0.0)
    assert rises[-1] <= -0.5
    assert rises == sorted(rises, key=lambda rise: (rise <= -0.5, abs(rise) > 0.3))

"""
Frames, rays and bounding boxes in the world's root frame.
"""

from __future__ import annotations

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.geometry import (
    ApproachFrame,
    BodyBounds,
    HorizontalFootprint,
    Ray,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    Box,
    CoordinateBounds,
    Scale,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.world_entity import Body

# %% frames


def test_an_approach_frame_points_its_x_axis_along_the_approach(kitchen_of_boxes):
    world = kitchen_of_boxes.world
    frame = ApproachFrame(
        position=np.array([1.0, 2.0, 3.0]),
        approach=np.array([0.0, 0.0, -2.0]),
        closing=np.array([0.0, 3.0, 0.0]),
    )

    matrix = frame.in_reference_frame(world.root).to_np()

    np.testing.assert_allclose(matrix[:3, 0], [0.0, 0.0, -1.0])
    np.testing.assert_allclose(matrix[:3, 1], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(matrix[:3, 2], np.cross([0, 0, -1.0], [0, 1.0, 0]))
    np.testing.assert_allclose(matrix[:3, 3], frame.position)


# %% footprints


def turned_box(*, scale: Scale, middle: np.ndarray, yaw: float) -> Body:
    """
    :param scale: The box's extents.
    :param middle: Where its middle lies.
    :param yaw: How far it is turned about the vertical, in radians.
    :return: The body of the box, fixed to the root of a world of its own.
    """
    world = World()
    with world.modify_world():
        root = Body(name=PrefixedName("kitchen"))
        world.add_kinematic_structure_entity(root)
        box = Body(
            name=PrefixedName("box"), collision=ShapeCollection([Box(scale=scale)])
        )
        world.add_connection(
            FixedConnection(
                parent=root,
                child=box,
                parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                    *middle, yaw=yaw, reference_frame=root
                ),
            )
        )
    return box


def test_a_turned_box_lies_along_its_long_side_and_across_its_narrow_side():
    yaw = 3 * np.pi / 4
    box = turned_box(
        scale=Scale(0.17, 0.04, 0.05), middle=np.array([1.0, 2.0, 0.9]), yaw=yaw
    )

    footprint = HorizontalFootprint.from_body(box)

    long_side = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    assert abs(footprint.long_axis @ long_side) == pytest.approx(1.0)
    assert footprint.narrow_axis @ long_side == pytest.approx(0.0, abs=1e-12)
    assert footprint.narrow_axis[2] == 0.0
    assert (footprint.length, footprint.width) == pytest.approx((0.17, 0.04))
    np.testing.assert_allclose(footprint.middle, [1.0, 2.0, 0.9], atol=1e-12)


# %% rays


def test_a_ray_enters_a_box_on_the_side_it_comes_from():
    bounds = CoordinateBounds(
        lower=np.array([0.0, 0.0, 0.0]), upper=np.array([2.0, 2.0, 2.0])
    )
    ray = Ray(point=np.array([1.0, 1.0, 1.0]), direction=np.array([1.0, 0.0, 0.0]))

    np.testing.assert_allclose(ray.entry_into(bounds), [0.0, 1.0, 1.0])


def test_a_slanted_ray_enters_a_box_through_the_side_it_reaches_first():
    bounds = CoordinateBounds(
        lower=np.array([0.0, 0.0, 0.0]), upper=np.array([4.0, 2.0, 2.0])
    )
    direction = np.array([1.0, 1.0, 0.0]) / np.sqrt(2)
    ray = Ray(point=np.array([2.0, 1.0, 1.0]), direction=direction)

    np.testing.assert_allclose(ray.entry_into(bounds), [1.0, 0.0, 1.0])


def test_a_point_lies_as_far_from_a_ray_as_from_the_line_it_runs_along():
    ray = Ray(point=np.array([1.0, 0.0, 0.0]), direction=np.array([2.0, 0.0, 0.0]))

    assert ray.distance_to(np.array([-3.0, 3.0, 4.0])) == 5.0


# %% bounding boxes


def test_a_body_is_bounded_by_its_collision_geometry(kitchen_of_boxes):
    bounds = BodyBounds(kitchen_of_boxes.world)
    half = kitchen_of_boxes.handle_scale.to_np() / 2

    box = bounds.of_body(kitchen_of_boxes.handle.root)

    np.testing.assert_allclose(box.lower, kitchen_of_boxes.handle_center - half)
    np.testing.assert_allclose(box.upper, kitchen_of_boxes.handle_center + half)


def test_the_center_of_a_body_is_the_center_of_its_box(kitchen_of_boxes):
    bounds = BodyBounds(kitchen_of_boxes.world)

    np.testing.assert_allclose(
        bounds.center_of(kitchen_of_boxes.plate), kitchen_of_boxes.plate_center
    )


def test_a_point_inside_a_body_lies_no_distance_from_it(kitchen_of_boxes):
    bounds = BodyBounds(kitchen_of_boxes.world)

    assert (
        bounds.distance_to(kitchen_of_boxes.plate, kitchen_of_boxes.plate_center) == 0
    )


def test_a_point_above_a_body_lies_as_far_from_it_as_above_its_top(kitchen_of_boxes):
    bounds = BodyBounds(kitchen_of_boxes.world)
    height_above = 0.5
    top = kitchen_of_boxes.plate_center[2] + kitchen_of_boxes.plate_scale.z / 2
    point = kitchen_of_boxes.plate_center.copy()
    point[2] = top + height_above

    assert np.isclose(bounds.distance_to(kitchen_of_boxes.plate, point), height_above)


def test_a_point_between_two_shapes_of_a_body_lies_as_far_as_the_nearer_one():
    world = World()
    with world.modify_world():
        body = Body(name=PrefixedName("two_cubes"))
        world.add_kinematic_structure_entity(body)
    cube_side, gap = 0.1, 1.0
    left_centre, right_centre = -0.5 * (gap + cube_side), 0.5 * (gap + cube_side)
    body.collision = ShapeCollection(
        [
            Box(
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=centre, reference_frame=body
                ),
                scale=Scale(cube_side, cube_side, cube_side),
            )
            for centre in (left_centre, right_centre)
        ],
        reference_frame=body,
    )
    distance_from_the_left_cube = 0.2
    point = np.array([left_centre + cube_side / 2 + distance_from_the_left_cube, 0, 0])

    bounds = BodyBounds(world)

    assert bounds.distance_to(body, point) == 0
    assert np.isclose(
        bounds.distance_to_nearest_shape(body, point), distance_from_the_left_cube
    )


def test_a_body_fixed_to_the_root_hangs_from_no_joint(kitchen_of_boxes):
    assert BodyBounds.joint_above(kitchen_of_boxes.plate) is None

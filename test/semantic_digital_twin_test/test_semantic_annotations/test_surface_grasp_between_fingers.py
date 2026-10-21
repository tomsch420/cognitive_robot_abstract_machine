"""
Tests for
:meth:`~semantic_digital_twin.grasping.surface_grasp.SurfaceGrasp.with_object_between_fingers`.
"""

import math

import pytest

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.grasping.surface_grasp import SurfaceGrasp
from semantic_digital_twin.semantic_annotations.semantic_annotations import Milk
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% fixtures


@pytest.fixture
def carton() -> Milk:
    """
    A box-shaped carton, 8 cm square and 20 cm tall, centred on its body's origin.
    """
    body = Body(
        name=PrefixedName("carton", prefix="between_fingers"),
        collision=ShapeCollection([Box(scale=Scale(0.08, 0.08, 0.2))]),
    )
    annotation = Milk(root=body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(annotation)
    return annotation


def half_width(carton: Milk) -> float:
    """
    :param carton: The carton.
    :return: Half its extent along its x-axis, in metres.
    """
    lowest, highest = carton.grasp_surface().bounds
    return float(highest[0] - lowest[0]) / 2


def from_above_along_x(depth: float, roll: float = 0.0) -> SurfaceGrasp:
    """
    :param depth: How far past the surface the gripper closes.
    :param roll: How far the closing direction is turned from the line of sight.
    :return: A grasp coming down from straight above onto the carton's face that looks
        along its x-axis.
    """
    return SurfaceGrasp(azimuth=0.0, height=0.5, depth=depth, pitch=0.0, roll=roll)


# %% fingers closing along the line of sight


def test_a_grasp_whose_fingers_already_straddle_the_object_keeps_its_depth(carton):
    """
    Fingers wide enough to have the object between them at the drawn depth leave it.
    """
    grasp = from_above_along_x(depth=half_width(carton))

    assert grasp.with_object_between_fingers(carton, 0.1) == grasp


def test_a_grasp_too_shallow_for_the_fingers_closes_just_deep_enough(carton):
    """
    The far finger would land inside the object, so the grasp closes deeper, by no more
    than it takes to bring the far finger past the object's far side.
    """
    finger_separation = 0.1
    grasp = from_above_along_x(depth=0.005)

    taken = grasp.with_object_between_fingers(carton, finger_separation)

    expected = 2 * half_width(carton) - finger_separation / 2
    assert taken.depth == pytest.approx(expected)


def test_an_object_wider_than_the_open_fingers_is_taken_at_its_middle(carton):
    """
    No depth has the object between fingers that stand narrower than it, so the grasp
    closes at its middle, where the fingers miss it by the same on either side.
    """
    grasp = from_above_along_x(depth=0.005)

    taken = grasp.with_object_between_fingers(carton, half_width(carton))

    assert taken.depth == pytest.approx(half_width(carton))


# %% fingers closing across the line of sight


def test_fingers_closing_across_the_line_of_sight_keep_the_drawn_depth(carton):
    """
    Closing deeper does not move fingers that close across the line of sight relative to
    the object, so the depth stays as drawn.
    """
    grasp = from_above_along_x(depth=0.005, roll=math.pi / 2)

    assert grasp.with_object_between_fingers(carton, 0.1) == grasp

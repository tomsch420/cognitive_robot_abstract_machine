"""
Fixtures for the tests of the PR2 performing RoboCasa's atomic tasks: small worlds of
boxes standing in for the parts of a kitchen.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pytest

from experiments.robocasa_atomic_tasks.panda_omron import PandaOmron
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import Handle
from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

from ...semantic_digital_twin_test.test_robocasa.conftest import pytest_configure

# %% a kitchen of boxes


@dataclass
class KitchenOfBoxes:
    """
    A world of a root and boxes fixed to it, each centred on its body's origin.
    """

    world: World
    """
    The world.
    """

    handle: Handle
    """
    A thin, tall handle.
    """

    plate: Body
    """
    A wide, flat plate.
    """

    handle_center: np.ndarray
    """
    Where the handle's centre lies, in the world's root frame.
    """

    handle_scale: Scale
    """
    The handle's extents.
    """

    plate_center: np.ndarray
    """
    Where the plate's centre lies.
    """

    plate_scale: Scale
    """
    The plate's extents.
    """


def box_body(
    world: World, root: Body, name: str, scale: Scale, center: np.ndarray
) -> Body:
    """
    :param world: The world the box is added to.
    :param root: The root of the world, which the box is fixed to.
    :param name: The name of the box's body.
    :param scale: The box's extents.
    :param center: Where its centre lies.
    :return: The box's body.
    """
    body = Body(name=PrefixedName(name), collision=ShapeCollection([Box(scale=scale)]))
    world.add_connection(
        FixedConnection(
            parent=root,
            child=body,
            parent_T_connection_expression=HomogeneousTransformationMatrix.from_xyz_rpy(
                *center, reference_frame=root
            ),
        )
    )
    return body


@pytest.fixture
def kitchen_of_boxes() -> KitchenOfBoxes:
    """
    A handle 2 cm deep, 4 cm wide and 20 cm tall at a counter's height a metre ahead,
    and a plate 30 cm across beside it.
    """
    world = World()
    handle_center = np.array([1.0, 0.0, 1.0])
    handle_scale = Scale(0.02, 0.04, 0.2)
    plate_center = np.array([1.0, 0.5, 0.9])
    plate_scale = Scale(0.3, 0.3, 0.02)
    with world.modify_world():
        root = Body(name=PrefixedName("kitchen"))
        world.add_kinematic_structure_entity(root)
        handle = Handle(
            root=box_body(world, root, "handle", handle_scale, handle_center)
        )
        world.add_semantic_annotation(handle)
        plate = box_body(world, root, "plate", plate_scale, plate_center)
    return KitchenOfBoxes(
        world=world,
        handle=handle,
        plate=plate,
        handle_center=handle_center,
        handle_scale=handle_scale,
        plate_center=plate_center,
        plate_scale=plate_scale,
    )


# %% RoboCasa's own robot


@pytest.fixture(scope="module")
def panda_omron() -> PandaOmron:
    """
    The PandaOmron, in a world of its own.
    """
    return PandaOmron.from_description()


# %% a kitchen RoboCasa composed nothing into


@dataclass
class EmptyRoboCasaKitchen:
    """
    Mimics a RoboCasa environment that holds no objects and no fixtures.
    """

    objects: dict[str, object] = field(default_factory=dict)
    """
    The objects of the task, by the names RoboCasa gives them.
    """

    fixtures: dict[str, object] = field(default_factory=dict)
    """
    The fixtures of the kitchen, by name.
    """

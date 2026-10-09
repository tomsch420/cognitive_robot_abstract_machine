"""
The mass of an annotation is the mass of the bodies it references, its parts included.
"""

import pytest

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Handle,
    Mug,
)
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.world_description.inertial_properties import Inertial
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

BODY_SCALE = Scale(0.1, 0.1, 0.1)
"""
Extents of the mug's body.
"""

HANDLE_SCALE = Scale(0.05, 0.1, 0.1)
"""
Extents of the mug's handle, half the body's volume.
"""

HANDLE_OFFSET = 0.075
"""
How far along x the handle's center is from the body's.
"""


def _box_body(name: str, scale: Scale, x: float = 0.0) -> Body:
    """
    :return: A body colliding as a box of ``scale`` centered ``x`` along its x-axis.
    """
    body = Body(name=PrefixedName(name, prefix="annotation_mass"))
    body.collision = ShapeCollection(
        [
            Box(
                origin=HomogeneousTransformationMatrix.from_xyz_rpy(
                    x=x, reference_frame=body
                ),
                scale=scale,
            )
        ],
        reference_frame=body,
    )
    return body


@pytest.fixture
def mug() -> Mug:
    """
    A mug whose box-shaped body has a box-shaped handle as a body of its own.
    """
    body = _box_body("mug", BODY_SCALE)
    handle_body = _box_body("handle", HANDLE_SCALE, x=HANDLE_OFFSET)
    mug = Mug(root=body)
    handle = Handle(root=handle_body)
    world = World()
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_connection(FixedConnection(parent=body, child=handle_body))
        world.add_semantic_annotation(mug)
        world.add_semantic_annotation(handle)
        mug.add(handle)
    return mug


def test_the_mass_of_an_annotation_includes_its_parts(mug):
    mug.root.inertial = Inertial(mass=0.3)
    mug.handle.root.inertial = Inertial(mass=0.1)

    assert mug.mass == pytest.approx(0.4)
    assert mug.handle.mass == pytest.approx(0.1)

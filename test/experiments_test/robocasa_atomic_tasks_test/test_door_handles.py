"""
Giving a RoboCasa door a handle of its own, split off the door's shape where RoboCasa
lets the handle collide as a cylinder.
"""

from __future__ import annotations

import numpy as np
import pytest
import trimesh
from numpy.typing import NDArray

from experiments.robocasa_atomic_tasks.door_handles import RoboCasaDoorHandleSplitter
from krrood_spatial_math.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.pipeline.mesh_decomposition.base import MeshDecomposer
from semantic_digital_twin.semantic_annotations.semantic_annotations import Door
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import (
    Box,
    Cylinder,
    Mesh,
    Scale,
    Shape,
)
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

# %% a door whose handle is part of its shape


class ConvexHullDecomposer(MeshDecomposer):
    """
    Decomposes a mesh into its convex hull: a quick stand-in for a real decomposer.
    """

    def apply_to_mesh(self, mesh: Mesh) -> list[Shape]:
        return [Mesh.from_trimesh(mesh=mesh.mesh.convex_hull, origin=mesh.origin)]

    def apply_to_mesh_and_save(self, mesh: Mesh, output_path: str) -> str:
        self.apply_to_mesh(mesh)[0].mesh.export(output_path, file_type="obj")
        return output_path


def door_T_bar() -> NDArray[np.float64]:
    """
    :return: Where the handle's bar lies on the door: 5 cm in front of it and 20 cm up,
        its axis along the door's width.
    """
    return HomogeneousTransformationMatrix.from_xyz_rpy(
        -0.05, 0.0, 0.2, roll=np.pi / 2
    ).to_np()


def bar() -> trimesh.Trimesh:
    """
    :return: The handle's bar, 2 cm thick and 40 cm long, where it lies on the door.
    """
    mesh = trimesh.creation.cylinder(radius=0.01, height=0.4, sections=16)
    mesh.apply_transform(door_T_bar())
    return mesh


def slab() -> trimesh.Trimesh:
    """
    :return: The door without its handle, 2 cm thick.
    """
    return trimesh.creation.box(extents=[0.02, 0.6, 0.5])


def door_in_a_world(*, collides_as_a_cylinder: bool = True) -> Door:
    """
    :param collides_as_a_cylinder: Whether the door's handle collides as a cylinder
        around the bar, as RoboCasa lets it.
    :return: A door, alone in its world, whose handle is part of its shape.
    """
    world = World()
    body = Body(name=PrefixedName("door"))
    body.visual = ShapeCollection(
        [
            Mesh.from_trimesh(
                mesh=trimesh.util.concatenate([slab(), bar()]),
                origin=HomogeneousTransformationMatrix(reference_frame=body),
            )
        ],
        reference_frame=body,
    )
    cylinder = Cylinder(
        width=0.02,
        height=0.4,
        origin=HomogeneousTransformationMatrix(door_T_bar(), reference_frame=body),
    )
    body.collision = ShapeCollection(
        [
            Box(
                scale=Scale(0.02, 0.6, 0.5),
                origin=HomogeneousTransformationMatrix(reference_frame=body),
            ),
            *([cylinder] if collides_as_a_cylinder else []),
        ],
        reference_frame=body,
    )
    door = Door(root=body)
    with world.modify_world():
        world.add_kinematic_structure_entity(body)
        world.add_semantic_annotation(door)
    return door


@pytest.fixture
def splitter() -> RoboCasaDoorHandleSplitter:
    """
    Splits a door's handle off, decomposing the shapes into their convex hulls.
    """
    return RoboCasaDoorHandleSplitter(decomposer=ConvexHullDecomposer())


def sorted_rows(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    :return: The points to the nanometre, ordered so that two sets of the same points
        compare equal.
    """
    rounded = np.round(points, 9)
    return rounded[np.lexsort(rounded.T[::-1])]


# %% finding the handle's faces


def test_the_handle_is_the_faces_around_its_collision_cylinder(
    splitter: RoboCasaDoorHandleSplitter,
):
    door = door_in_a_world()

    face_centers = splitter.handle_face_centers(door=door)

    np.testing.assert_allclose(
        sorted_rows(face_centers), sorted_rows(bar().triangles_center), atol=1e-9
    )


def test_a_door_without_a_collision_cylinder_has_no_handle_faces(
    splitter: RoboCasaDoorHandleSplitter,
):
    door = door_in_a_world(collides_as_a_cylinder=False)

    assert splitter.handle_face_centers(door=door) is None


# %% splitting the handle off


def test_the_handle_is_split_off_into_a_body_of_its_own(
    splitter: RoboCasaDoorHandleSplitter,
):
    door = door_in_a_world()

    handle = splitter.split_handle_off(door=door)

    assert door.handle is handle
    assert handle.root is not door.root
    assert len(handle.root.visual.combined_mesh.faces) == len(bar().faces)
    assert len(door.root.visual.combined_mesh.faces) == len(slab().faces)


def test_a_door_with_a_handle_is_not_split_again(
    splitter: RoboCasaDoorHandleSplitter,
):
    door = door_in_a_world()
    handle = splitter.split_handle_off(door=door)

    assert splitter.split_handle_off(door=door) is None
    assert door.handle is handle
    assert len(door._world.bodies) == 2


def test_a_door_without_a_collision_cylinder_is_not_split(
    splitter: RoboCasaDoorHandleSplitter,
):
    door = door_in_a_world(collides_as_a_cylinder=False)

    assert splitter.split_handle_off(door=door) is None
    assert door.handle is None
    assert len(door._world.bodies) == 1

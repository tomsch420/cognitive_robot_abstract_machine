"""
Splitting a part, such as a handle or a rim, off the shape of an object into a body of
its own, annotated as that part of the object's annotation.

A finder decides which piece of the object's shape the part is; the pipeline step cuts
the shape there and gives each piece its own body. Shapes are taken in the frame of the object's root body, its z-axis
pointing up.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np
import trimesh
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING, Optional, Type

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.pipeline.pipeline import Step
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix
from semantic_digital_twin.world_description.connections import FixedConnection
from semantic_digital_twin.world_description.geometry import Mesh
from semantic_digital_twin.world_description.inertial_properties import Inertial
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import (
        HasRootBody,
        PartWholeRelationship,
    )
    from semantic_digital_twin.world import World

# %% splitting a shape


@dataclass
class ShapeSplit:
    """
    A shape cut into a part and the rest of it.
    """

    part: trimesh.Trimesh
    """
    The part.
    """

    rest: trimesh.Trimesh
    """
    The rest of the shape.
    """

    @classmethod
    def by_faces(
        cls, shape: trimesh.Trimesh, in_part: NDArray[np.bool_]
    ) -> Optional[ShapeSplit]:
        """
        :param shape: The shape to split.
        :param in_part: For each face of ``shape``, whether it belongs to the part.
        :return: The faces of the part and the remaining faces; ``None`` if either
            would be empty.
        """
        if not in_part.any() or in_part.all():
            return None
        return cls(
            part=shape.submesh([np.flatnonzero(in_part)], append=True),
            rest=shape.submesh([np.flatnonzero(~in_part)], append=True),
        )

    @classmethod
    def by_plane(
        cls,
        shape: trimesh.Trimesh,
        point: NDArray[np.float64],
        normal: NDArray[np.float64],
    ) -> Optional[ShapeSplit]:
        """
        Cut the shape along a plane. A closed shape is cut into closed pieces.

        :param shape: The shape to split.
        :param point: A point on the plane.
        :param normal: The plane's normal, pointing towards the part.
        :return: What lies on the side of the plane the normal points to, as the part,
            and what lies on the other side; ``None`` if either would be empty.
        """
        cap = shape.is_watertight
        part = shape.slice_plane(point, normal, cap=cap)
        rest = shape.slice_plane(point, -np.asarray(normal), cap=cap)
        if part is None or rest is None or part.is_empty or rest.is_empty:
            return None
        return cls(part=part, rest=rest)


class PartFinder(ABC):
    """
    Finds the piece of an object's shape that is one of its parts.
    """

    @abstractmethod
    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        """
        :param shape: The object's shape, in its root body's frame.
        :return: The shape split into the part and the rest; ``None`` if the shape has
            no such part.
        """


# %% the pipeline step


@dataclass
class SplitPartFromShape(Step):
    """
    Gives every annotation of a type the part its shape has, as a body of its own.

    The piece of the annotation's root body that the finder finds is moved to a new
    body, fixed to the root body at its origin, and annotated as the part. Both bodies
    look and collide as their own piece of the shape; a physics simulator that needs
    convex collision shapes decomposes them afterwards.

    The root body keeps its inertial properties, which describe the whole object, and
    the part's body is given a negligible one, so the object weighs and turns as it did
    before the split.
    """

    whole_type: Type[PartWholeRelationship]
    """
    The type of annotation whose shapes are split.
    """

    part_type: Type[HasRootBody]
    """
    The type of annotation the part is given.
    """

    finder: PartFinder
    """
    Finds the part in each annotation's shape.
    """

    def _apply(self, world: World) -> World:
        for whole in world.get_semantic_annotations_by_type(self.whole_type):
            self.split(whole)
        return world

    def split(self, whole: PartWholeRelationship) -> Optional[HasRootBody]:
        """
        Split the part off the shape of one annotation.

        :param whole: The annotation whose root body's shape is split.
        :return: The part, added to the annotation and its world; ``None`` if the finder
            finds none or the root body has no shape.
        """
        root = whole.root
        geometry = root.visual or root.collision
        if not geometry:
            return None
        split = self.finder.split(geometry.combined_mesh)
        if split is None:
            return None
        collides = bool(root.collision)
        root.visual = self._shapes(split.rest, root)
        if collides:
            root.collision = self._shapes(split.rest, root)
        body = Body(
            name=PrefixedName(
                f"{root.name.name}_{self.part_type.__name__.lower()}",
                root.name.prefix,
            ),
            inertial=Inertial.negligible(),
        )
        body.visual = self._shapes(split.part, body)
        if collides:
            body.collision = self._shapes(split.part, body)
        part = self.part_type(root=body)
        world = root._world
        with world.modify_world():
            world.add_connection(FixedConnection(parent=root, child=body))
            world.add_semantic_annotation(part)
            whole.add(part)
        return part

    @staticmethod
    def _shapes(mesh: trimesh.Trimesh, body: Body) -> ShapeCollection:
        """
        :param mesh: A piece of the shape, in the frame of ``body``.
        :param body: The body the piece belongs to.
        :return: The piece as the shapes of ``body``.
        """
        return ShapeCollection(
            [
                Mesh.from_trimesh(
                    mesh=mesh,
                    origin=HomogeneousTransformationMatrix(reference_frame=body),
                )
            ],
            reference_frame=body,
        )

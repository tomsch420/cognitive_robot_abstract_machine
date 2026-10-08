"""
Finding the part of an open container's shape that is its rim, so that the container's
annotation can be given a
:class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Rim` to be
grasped at.

Shapes are taken in the frame of the container's root body, its z-axis pointing up.
"""

from __future__ import annotations

from dataclasses import dataclass

import trimesh
import trimesh.remesh
from typing_extensions import Optional

from semantic_digital_twin.datastructures.definitions import Axis


@dataclass
class RimFinder:
    """
    Finds the rim of an open container: the band of its shape just below its highest
    point.
    """

    depth: float = 0.02
    """
    How far below the container's highest point the rim reaches, in meters.
    """

    piece_length: float = 0.002
    """
    How long the pieces of the faces crossing the rim's lower edge may be, in meters;
    the rim's lower edge is as ragged as this.
    """

    def find(self, shape: trimesh.Trimesh) -> Optional[trimesh.Trimesh]:
        """
        :param shape: The container's shape, in its root body's frame.
        :return: The part of ``shape`` within :attr:`depth` of its highest point, with
            the faces crossing that depth cut into pieces no longer than
            :attr:`piece_length`; ``None`` if the shape is empty.
        """
        if shape.is_empty:
            return None
        lowest_rim_height = shape.bounds[1][Axis.Z] - self.depth
        reaching_up = shape.triangles[:, :, Axis.Z].max(axis=1) >= lowest_rim_height
        vertices, faces = trimesh.remesh.subdivide_to_size(
            shape.vertices,
            shape.faces[reaching_up],
            max_edge=self.piece_length,
            max_iter=20,
        )
        pieces = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        within = pieces.triangles_center[:, Axis.Z] >= lowest_rim_height
        return pieces.submesh([within.nonzero()[0]], append=True)

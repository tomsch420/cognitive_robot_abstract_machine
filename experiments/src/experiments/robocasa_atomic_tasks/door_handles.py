"""
Giving the doors of a RoboCasa kitchen a handle of their own, which RoboCasa models as
part of the door's shape.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from semantic_digital_twin.part_finding.locating import LocatedFacesFinder
from semantic_digital_twin.part_finding.splitting import PartSplitter
from semantic_digital_twin.pipeline.mesh_decomposition.base import MeshDecomposer
from semantic_digital_twin.pipeline.mesh_decomposition.vhacd import VHACDMeshDecomposer
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Door,
    Handle,
)
from semantic_digital_twin.world_description.geometry import Cylinder

# %% splitting a door's handle off


@dataclass(kw_only=True)
class RoboCasaDoorHandleSplitter:
    """
    Splits the handle off a RoboCasa door into a body of its own, fixed to the door and
    annotated as the door's handle.

    The handle is the part of the door's look that lies around the cylinder RoboCasa
    lets the handle collide as.
    """

    margin: float = 0.01
    """
    How far outside the handle's collision cylinder a face of the door's look may reach
    and still belong to the handle, in meters.
    """

    decomposer: MeshDecomposer = field(
        default_factory=lambda: VHACDMeshDecomposer(max_convex_hulls=32)
    )
    """
    Gives the handle and the rest of the door new collision shapes from their looks.
    """

    def split_handle_off(self, *, door: Door) -> Handle | None:
        """
        :param door: A door of a RoboCasa kitchen.
        :return: The door's new handle; ``None`` if the door has a handle already or
            RoboCasa lets none of it collide as a cylinder.
        """
        if door.handle is not None:
            return None
        face_centers = self.handle_face_centers(door=door)
        if face_centers is None:
            return None
        return PartSplitter(
            part_type=Handle,
            finder=LocatedFacesFinder(face_centers=face_centers),
            decomposer=self.decomposer,
        ).split(whole=door)

    def handle_face_centers(self, *, door: Door) -> NDArray[np.float64] | None:
        """
        :param door: A door of a RoboCasa kitchen.
        :return: The centers of the faces of the door's look that belong to its handle,
            in the door's frame; ``None`` if RoboCasa lets no part of the door collide as
            a cylinder.
        """
        cylinders = [
            shape for shape in door.root.collision if isinstance(shape, Cylinder)
        ]
        if not cylinders:
            return None
        (cylinder,) = cylinders
        look = door.root.visual.combined_mesh
        cylinder_T_door = np.linalg.inv(cylinder.origin.to_np())
        vertices = (
            cylinder_T_door @ np.c_[look.vertices, np.ones(len(look.vertices))].T
        ).T[:, :3]
        around_the_cylinder = (
            np.hypot(vertices[:, 0], vertices[:, 1]) <= cylinder.width / 2 + self.margin
        ) & (np.abs(vertices[:, 2]) <= cylinder.height / 2 + self.margin)
        handle_faces = around_the_cylinder[look.faces].all(axis=1)
        return look.triangles_center[handle_faces]

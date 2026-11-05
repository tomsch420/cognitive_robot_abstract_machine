from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from typing_extensions import TYPE_CHECKING

from krrood_spatial_math.spatial_types import (
    HomogeneousTransformationMatrix,
    Point3,
    RotationMatrix,
    Vector3,
)
from semantic_digital_twin.exceptions import GraspNotOnObjectError
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.grasping.object_relative_grasp import ObjectRelativeGrasp

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% grasps on an object's surface


@dataclass
class SurfaceGrasp(ObjectRelativeGrasp):
    """
    A grasp placed on the surface of an object around its root body's z-axis.

    With ``root_V_outward`` the horizontal direction at :attr:`yaw`, ``root_P_surface``
    is the first point of the grasp surface that a horizontal ray hits when it comes
    from ``root_V_outward`` at :attr:`height`. The gripper closes at
    ``root_P_grasp = root_P_surface - depth * root_V_outward``. The x-axis of
    ``root_T_grasp`` is the direction of approach, straight down tilted by :attr:`pitch`
    towards ``-root_V_outward``; its y-axis is the closing direction, ``root_V_outward``
    made perpendicular to the approach and turned by :attr:`roll` about it.
    """

    yaw: float
    """
    The yaw of ``root_V_outward`` about the root body's z-axis, in radians.
    """

    height: float
    """
    The height of ``root_P_surface``, as a fraction of the grasp surface's bounding box
    from its bottom to its top.
    """

    depth: float
    """
    How far behind ``root_P_surface`` the gripper closes, in meters.
    """

    pitch: float
    """
    How far the direction of approach is tilted from straight down, in radians.
    """

    roll: float
    """
    How far the closing direction is turned about the direction of approach, in radians.
    """

    def grasp_candidate(self, graspable: HasGraspCandidates) -> GraspCandidate:
        root_V_outward = self._outward_direction()
        root_P_surface = self._surface_point(graspable, root_V_outward)
        if root_P_surface is None:
            raise GraspNotOnObjectError(grasp=self, graspable=graspable)
        root_V_approach = self._approach_direction(root_V_outward)
        root_V_closing = self._closing_direction(root_V_approach, root_V_outward)
        root_T_grasp = HomogeneousTransformationMatrix.from_point_rotation_matrix(
            point=root_P_surface - root_V_outward * self.depth,
            rotation_matrix=RotationMatrix.from_vectors(
                x=root_V_approach, y=root_V_closing
            ),
            reference_frame=graspable.root,
        )
        return GraspCandidate(graspable, root_T_grasp.pose)

    def with_object_between_fingers(
        self, graspable: HasGraspCandidates, finger_separation: float
    ) -> SurfaceGrasp:
        """
        :param graspable: The object to take hold of.
        :param finger_separation: How far apart the fingers stand as they close in, in
            meters.
        :return: A copy of this grasp at the depth nearest its own at which fingers that
            far apart have the object's bounding box between them along the closing
            direction; at the object's middle where it is wider than that.
        :raises GraspNotOnObjectError: If the object's grasp surface does not reach
            ``root_P_surface``.
        """
        root_V_outward = self._outward_direction()
        root_P_surface = self._surface_point(graspable, root_V_outward)
        if root_P_surface is None:
            raise GraspNotOnObjectError(grasp=self, graspable=graspable)
        outward = root_V_outward.to_np()[:3]
        closing = self._closing_direction(
            self._approach_direction(root_V_outward), root_V_outward
        ).to_np()[:3]
        surface_point = root_P_surface.to_np()[:3]
        shallower_per_meter_deeper = float(closing @ outward)
        if np.isclose(shallower_per_meter_deeper, 0.0):
            return self

        lowest, highest = graspable.grasp_surface().bounds
        middle, half_extent = (lowest + highest) / 2, (highest - lowest) / 2
        object_middle = float(closing @ middle)
        object_reach = float(np.abs(closing) @ half_extent)
        finger_reach = finger_separation / 2
        surface_along_closing = float(closing @ surface_point)
        drawn = surface_along_closing - self.depth * shallower_per_meter_deeper
        if object_reach > finger_reach:
            between_fingers = object_middle
        else:
            between_fingers = float(
                np.clip(
                    drawn,
                    object_middle + object_reach - finger_reach,
                    object_middle - object_reach + finger_reach,
                )
            )
        depth = (surface_along_closing - between_fingers) / shallower_per_meter_deeper
        through_the_object = float(
            outward @ surface_point - (outward @ middle - np.abs(outward) @ half_extent)
        )
        return replace(self, depth=float(np.clip(depth, 0.0, through_the_object)))

    def reaches_surface_of(self, graspable: HasGraspCandidates) -> bool:
        """
        :return: Whether the object's grasp surface reaches ``root_P_surface``.
        """
        return self._surface_point(graspable, self._outward_direction()) is not None

    def _outward_direction(self) -> Vector3:
        """
        :return: ``root_V_outward``.
        """
        return Vector3(math.cos(self.yaw), math.sin(self.yaw), 0.0)

    def _approach_direction(self, root_V_outward: Vector3) -> Vector3:
        """
        :param root_V_outward: The horizontal direction ``root_P_surface`` is seen from.
        :return: The direction of approach, straight down tilted by :attr:`pitch`
            towards ``-root_V_outward``.
        """
        return Vector3.NEGATIVE_Z() * math.cos(self.pitch) - root_V_outward * math.sin(
            self.pitch
        )

    def _closing_direction(
        self, root_V_approach: Vector3, root_V_outward: Vector3
    ) -> Vector3:
        """
        :param root_V_approach: The direction of approach.
        :param root_V_outward: The horizontal direction ``root_P_surface`` is seen from.
        :return: ``root_V_outward`` made perpendicular to ``root_V_approach`` and turned
            by :attr:`roll` about it.
        """
        root_V_across = root_V_outward - root_V_approach * root_V_outward.dot(
            root_V_approach
        )
        root_V_across.scale(1)
        return root_V_across * math.cos(self.roll) + root_V_approach.cross(
            root_V_across
        ) * math.sin(self.roll)

    def _surface_point(
        self, graspable: HasGraspCandidates, root_V_outward: Vector3
    ) -> Point3 | None:
        """
        :param graspable: The object whose grasp surface the ray is cast against.
        :param root_V_outward: The horizontal direction the ray comes from.
        :return: ``root_P_surface``; ``None`` if the ray hits nothing.
        """
        mesh = graspable.grasp_surface()
        outward = root_V_outward.to_np()[:3]
        lowest, highest = mesh.bounds
        origin = (lowest + highest) / 2 + outward * float(
            np.linalg.norm(highest - lowest)
        )
        origin[2] = lowest[2] + self.height * (highest[2] - lowest[2])
        locations, _, _ = mesh.ray.intersects_location(
            ray_origins=origin[None, :], ray_directions=-outward[None, :]
        )
        if len(locations) == 0:
            return None
        return Point3(
            *locations[int(np.argmin(np.linalg.norm(locations - origin, axis=1)))]
        )

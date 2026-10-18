"""
Grasps described by where on an object's surface they take hold and how the gripper
comes in, statements of where an object may be grasped, and what happened when one was
tried.

Every field of :class:`SurfaceGrasp` and :class:`GraspResult` is a number, a truth value,
a vector or a rotation, so a probabilistic model can be learned over tried grasps. Where
an object may be grasped is stated as an underspecified statement over
:class:`SurfaceGrasp`; a model registry answers it, knowing nothing but the statement at
first and what was learned later.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation
from typing_extensions import TYPE_CHECKING, List, Optional

from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.entity_query_language.factories import ConditionType, a
from krrood.entity_query_language.query.match import Match
from krrood.parametrization.model_registries import (
    ModelRegistry,
    UniformPriorRegistry,
)
from semantic_digital_twin.exceptions import SurfaceGraspNotOnSurfaceError
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.spatial_types import (
    HomogeneousTransformationMatrix,
    Vector3,
)
from semantic_digital_twin.spatial_types.spatial_types import AxisAngle, Quaternion

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% the result of trying a grasp


@dataclass
class GraspResult:
    """
    What happened to an object when a robot tried to pick it up by a grasp.

    The object's displacement and rotation compare where it rested before the grasp
    with where it is held at the end, along the world's axes, so that a use that cares
    about another goal than lifting, such as pulling the object out of a shelf, can ask
    for it.

    The slip compares where the object sits in the gripper once the fingers have closed
    on it with where it sits after it was lifted and held: an object held firmly does
    not move relative to the fingers at all, one that was never held stays behind by the
    whole lift. It is measured along the grasp's own axes, the direction of approach,
    the closing direction and their normal, so that it means the same for every robot.

    A rotation is stated as :func:`axis_angle_of` states it: an angle in [0, pi] about a
    unit axis, no rotation being the angle zero about the z-axis.

    The vectors and rotations carry no reference frame; each field says which axes it
    is measured along.
    """

    object_raised: bool
    """
    Whether the object ended up clear of where it rested.
    """

    motion_completed: bool
    """
    Whether the robot went through every step of the pick-up within its time limit.
    """

    object_displacement: Vector3
    """
    How far and in which direction the object moved from where it rested to where it
    was held at the end, in meters, along the world's axes.
    """

    object_rotation: AxisAngle
    """
    How the object turned from where it rested to where it was held at the end, its
    axis along the world's axes.
    """

    translational_slip: Vector3
    """
    How far and in which direction the object moved relative to the grasp while it was
    lifted and held, in meters, along the grasp's axes.
    """

    rotational_slip: AxisAngle
    """
    How the object turned relative to the grasp while it was lifted and held, its axis
    along the grasp's axes.
    """

    @property
    def lifted(self) -> bool:
        """
        :return: Whether the object ended up clear of where it rested and the robot went
            through every step of the pick-up.
        """
        return self.object_raised and self.motion_completed


# %% grasps on an object's surface


@dataclass
class SurfaceGrasp:
    """
    A grasp described relative to the surface of the object it takes, and independent of
    the robot taking it.

    The point to take hold of lies on the outside of the object's grasp surface: looking
    horizontally at the object's vertical axis from :attr:`azimuth`, at :attr:`height`
    of its bounding box, it is the first point of the surface in that line of sight. The
    gripper's center closes :attr:`depth` further along the line of sight, so that its
    fingers straddle a wall as thick as twice that, or reach that far along a handle
    seen from its end.

    The gripper comes in from straight above when :attr:`pitch` is zero and tilts
    towards the line of sight as it grows. With :attr:`roll` zero the fingers close
    along the line of sight; a roll turns the closing direction about the direction of
    approach.

    The line of sight, not the surface normal, sets these directions, so that they stay
    meaningful where a scanned mesh's normals are noisy or where the line meets the
    surface at a slant.
    """

    azimuth: float
    """
    The direction around the object's vertical axis from which the point is found, in
    radians.
    """

    height: float
    """
    The height of the point, as a fraction of the object's bounding box from its bottom
    to its top.
    """

    depth: float
    """
    How far beyond the surface point, along the line of sight, the gripper's center
    closes, in meters.
    """

    pitch: float
    """
    How far the direction of approach is tilted from straight down towards the line of
    sight, in radians; below a quarter turn, where the approach would be horizontal.
    """

    roll: float
    """
    How far the closing direction is turned about the direction of approach, in radians.
    """

    result: Optional[GraspResult] = field(default=None)
    """
    What happened when the grasp was tried; ``None`` while it has not been.
    """

    def grasp_candidate(self, graspable: HasGraspCandidates) -> GraspCandidate:
        """
        :param graspable: The object to take hold of.
        :return: This grasp as a grasp frame on the object's root body, its x-axis
            the direction of approach and its y-axis the closing direction; these are
            the grasp's axes a :class:`GraspResult` measures slip along.
        :raises SurfaceGraspNotOnSurfaceError: If the object's grasp surface does not
            reach the point this grasp names.
        """
        outward = self._line_of_sight()
        surface_point = self._surface_point(graspable, outward)
        if surface_point is None:
            raise SurfaceGraspNotOnSurfaceError(surface_grasp=self, graspable=graspable)
        approach = self._approach_direction(outward)
        closing = self._closing_direction(approach, outward)
        root_T_grasp = np.eye(4)
        root_T_grasp[:3, 0] = approach
        root_T_grasp[:3, 1] = closing
        root_T_grasp[:3, 2] = np.cross(approach, closing)
        root_T_grasp[:3, 3] = surface_point - outward * self.depth
        return GraspCandidate(
            graspable,
            HomogeneousTransformationMatrix(
                root_T_grasp, reference_frame=graspable.root
            ).pose,
        )

    def reaches_surface_of(self, graspable: HasGraspCandidates) -> bool:
        """
        :param graspable: The object to take hold of.
        :return: Whether the object's grasp surface reaches the point this grasp names.
        """
        return self._surface_point(graspable, self._line_of_sight()) is not None

    def _line_of_sight(self) -> NDArray[np.float64]:
        """
        :return: The horizontal direction the surface point is seen from.
        """
        return np.array([math.cos(self.azimuth), math.sin(self.azimuth), 0.0])

    def _surface_point(
        self, graspable: HasGraspCandidates, outward: NDArray[np.float64]
    ) -> Optional[NDArray[np.float64]]:
        """
        Cast a horizontal ray from outside the object towards its vertical axis.

        :param graspable: The object whose grasp surface the ray is cast against.
        :param outward: The horizontal direction the ray comes from.
        :return: The first point the ray hits, in the object's root frame; ``None`` if
            it hits nothing.
        """
        mesh = graspable.grasp_surface()
        lowest, highest = mesh.bounds
        center = (lowest + highest) / 2
        origin = center + outward * float(np.linalg.norm(highest - lowest))
        origin[2] = lowest[2] + self.height * (highest[2] - lowest[2])
        locations, _, _ = mesh.ray.intersects_location(
            ray_origins=origin[None, :], ray_directions=-outward[None, :]
        )
        if len(locations) == 0:
            return None
        return locations[int(np.argmin(np.linalg.norm(locations - origin, axis=1)))]

    def _approach_direction(self, outward: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        :param outward: The horizontal direction the surface point was seen from.
        :return: The direction the gripper travels in towards the grasp.
        """
        down = np.array([0.0, 0.0, -1.0])
        return math.cos(self.pitch) * down - math.sin(self.pitch) * outward

    def _closing_direction(
        self, approach: NDArray[np.float64], outward: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """
        :param approach: The direction the gripper travels in.
        :param outward: The horizontal direction the surface point was seen from.
        :return: The direction the fingers close along: the line of sight made
            perpendicular to the approach, turned by :attr:`roll` about it.
        """
        across = outward - np.dot(outward, approach) * approach
        across /= np.linalg.norm(across)
        return math.cos(self.roll) * across + math.sin(self.roll) * np.cross(
            approach, across
        )


# %% stating where an object may be grasped


def any_surface_grasp(require_lifting: bool = False) -> Match[SurfaceGrasp]:
    """
    :param require_lifting: Whether to ask only for grasps that lift the object, by
        stating the grasp's result as well; only a model learned over tried grasps and
        their results can answer that.
    :return: A statement asking for a surface grasp with every parameter left free, for
        an annotation to narrow down with ``where``.
    """
    if require_lifting:
        result = a(GraspResult)(
            object_raised=True,
            motion_completed=True,
            object_displacement=_any_vector(),
            object_rotation=_any_rotation(),
            translational_slip=_any_vector(),
            rotational_slip=_any_rotation(),
        )
    else:
        result = None
    return a(SurfaceGrasp)(
        azimuth=..., height=..., depth=..., pitch=..., roll=..., result=result
    )


def _any_vector() -> Match[Vector3]:
    """
    :return: A statement asking for a vector with every component left free.
    """
    return a(Vector3)(x=..., y=..., z=...)


def _any_rotation() -> Match[AxisAngle]:
    """
    :return: A statement asking for a rotation with its axis and angle left free.
    """
    return a(AxisAngle)(axis=_any_vector(), angle=...)


def axis_angle_of(rotation: NDArray[np.float64]) -> AxisAngle:
    """
    :param rotation: A rotation matrix.
    :return: The rotation as an angle in [0, pi] about a unit axis. No rotation is the
        angle zero about the z-axis, as
        :meth:`~semantic_digital_twin.spatial_types.spatial_types.AxisAngle.from_quaternion`
        reads it.
    """
    x, y, z, w = Rotation.from_matrix(rotation).as_quat(canonical=True)
    axis_angle = AxisAngle.from_quaternion(Quaternion(x=x, y=y, z=z, w=w))
    return AxisAngle(
        axis=Vector3(*axis_angle.axis.to_np()[:3]), angle=float(axis_angle.angle)
    )


def from_any_side(grasp: Match[SurfaceGrasp]) -> List[ConditionType]:
    """
    :param grasp: A statement's grasp.
    :return: The conditions that the grasp comes from any direction around the object,
        approaches it up to about seventy degrees off vertical, staying below a quarter
        turn where the approach would be horizontal, and closes up to an eighth of a
        turn off the line of sight.
    """
    return [
        grasp.azimuth >= 0.0,
        grasp.azimuth < 2 * math.pi,
        grasp.pitch >= 0.0,
        grasp.pitch < 1.2,
        grasp.roll >= -math.pi / 4,
        grasp.roll < math.pi / 4,
    ]


def draw_surface_grasps(
    statement: Match[SurfaceGrasp],
    number_of_grasps: int,
    model_registry: Optional[ModelRegistry] = None,
) -> List[SurfaceGrasp]:
    """
    :param statement: A statement of where an object may be grasped.
    :param number_of_grasps: How many grasps to draw, as the statement's limit.
    :param model_registry: Answers the statement; ``None`` draws uniformly from what it
        allows, which cannot answer a statement that requires lifting.
    :return: Grasps answering the statement.
    """
    return list(
        statement.limit(number_of_grasps).evaluate(
            backend=ProbabilisticBackend(model_registry or UniformPriorRegistry())
        )
    )

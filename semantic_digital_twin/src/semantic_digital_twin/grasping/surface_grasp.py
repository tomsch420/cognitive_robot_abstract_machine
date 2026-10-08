"""
Grasps described by where on an object's surface they take hold and how the gripper
comes in, the regions of such grasps an object allows, and what happened when one was
tried.

Every field of :class:`SurfaceGrasp` and :class:`GraspResult` is a plain number or truth
value, so a probabilistic model can be learned over tried grasps. Where an object may be
grasped is stated as an underspecified statement over :class:`SurfaceGrasp`; a model
registry answers it, knowing nothing but the statement at first and what was learned
later.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from typing_extensions import TYPE_CHECKING, List, Optional

from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.entity_query_language.factories import a, and_, or_
from krrood.entity_query_language.query.match import Match
from random_events.interval import Bound, Interval, closed_open
from krrood.parametrization.model_registries import (
    ModelRegistry,
    UniformPriorRegistry,
)
from semantic_digital_twin.exceptions import SurfaceGraspNotOnSurfaceError
from semantic_digital_twin.grasping.grasp_candidates import GraspCandidate
from semantic_digital_twin.spatial_types import HomogeneousTransformationMatrix

if TYPE_CHECKING:
    from semantic_digital_twin.semantic_annotations.mixins import HasGraspCandidates

# %% the result of trying a grasp


@dataclass
class GraspResult:
    """
    What happened to an object when a robot tried to pick it up by a grasp.

    The slip compares where the object sits in the gripper once the fingers have closed
    on it with where it sits after it was lifted and held: an object held firmly does
    not move relative to the fingers at all, one that was never held stays behind by the
    whole lift.
    """

    lifted: bool
    """
    Whether the object ended up clear of where it rested, whether or not the motion
    completed.
    """

    object_rise: float
    """
    How far above where it rested the object ended up, in meters.
    """

    translational_slip: float
    """
    How far the object moved relative to the gripper while it was lifted and held, in
    meters.
    """

    rotational_slip: float
    """
    How far the object turned relative to the gripper while it was lifted and held, in
    radians.
    """

    motion_completed: bool
    """
    Whether the robot went through every step of the pick-up within its time limit.
    """


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
        :return: This grasp as a grasp frame on the object's root body.
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


# %% where an object may be grasped


@dataclass
class SurfaceGraspRegion:
    """
    One region of grasps an object allows: an interval for every parameter of a
    :class:`SurfaceGrasp`, each closed at its lower and open at its upper end unless
    built otherwise.

    Which heights and depths make sense depends on the object's shape, so they have no
    defaults. The other defaults cover every direction around the object, approaches up
    to about seventy degrees off vertical and closing directions up to an eighth of a
    turn off the line of sight.
    """

    height: Interval
    """
    See :attr:`SurfaceGrasp.height`.
    """

    depth: Interval
    """
    See :attr:`SurfaceGrasp.depth`.
    """

    azimuth: Interval = field(default_factory=lambda: closed_open(0.0, 2 * math.pi))
    """
    See :attr:`SurfaceGrasp.azimuth`.
    """

    pitch: Interval = field(default_factory=lambda: closed_open(0.0, 1.2))
    """
    See :attr:`SurfaceGrasp.pitch`; stays below a quarter turn, where the approach would
    be horizontal.
    """

    roll: Interval = field(
        default_factory=lambda: closed_open(-math.pi / 4, math.pi / 4)
    )
    """
    See :attr:`SurfaceGrasp.roll`.
    """

    def condition(self, grasp: Match):
        """
        :param grasp: The statement's grasp variable.
        :return: The condition that the grasp lies within this region.
        """
        return and_(
            *(
                self._lies_within(attribute, interval)
                for attribute, interval in (
                    (grasp.azimuth, self.azimuth),
                    (grasp.height, self.height),
                    (grasp.depth, self.depth),
                    (grasp.pitch, self.pitch),
                    (grasp.roll, self.roll),
                )
            )
        )

    @staticmethod
    def _lies_within(attribute, interval: Interval):
        """
        :param attribute: An attribute of the statement's grasp variable.
        :param interval: The values the attribute may take.
        :return: The condition that the attribute lies within the interval.
        """
        conditions = [
            and_(
                (
                    attribute >= simple_interval.lower
                    if simple_interval.left == Bound.CLOSED
                    else attribute > simple_interval.lower
                ),
                (
                    attribute <= simple_interval.upper
                    if simple_interval.right == Bound.CLOSED
                    else attribute < simple_interval.upper
                ),
            )
            for simple_interval in interval.simple_sets
        ]
        return conditions[0] if len(conditions) == 1 else or_(*conditions)


@dataclass
class SurfaceGraspStatement:
    """
    The underspecified statement that a grasp lies in one of several regions.
    """

    regions: List[SurfaceGraspRegion]
    """
    The regions a grasp may lie in.
    """

    require_lifting: bool = False
    """
    Whether to ask only for grasps that lift the object, by stating the grasp's result
    as well; only a model learned over tried grasps and their results can answer that.
    """

    def match(self) -> Match:
        """
        :return: The statement asking for a surface grasp within one of the regions,
            leaving every parameter free, and its result too if :attr:`require_lifting`
            asks for a lifting one.
        """
        parameters = dict(azimuth=..., height=..., depth=..., pitch=..., roll=...)
        if self.require_lifting:
            parameters["result"] = a(GraspResult)(
                lifted=True,
                object_rise=...,
                translational_slip=...,
                rotational_slip=...,
                motion_completed=...,
            )
        grasp = a(SurfaceGrasp)(**parameters)
        conditions = [region.condition(grasp) for region in self.regions]
        grasp.where(conditions[0] if len(conditions) == 1 else or_(*conditions))
        return grasp

    def draw(
        self,
        number_of_grasps: int,
        model_registry: Optional[ModelRegistry] = None,
    ) -> List[SurfaceGrasp]:
        """
        :param number_of_grasps: How many grasps to draw.
        :param model_registry: Answers the statement; ``None`` draws uniformly within
            the regions, which cannot answer a statement that requires lifting.
        :return: Grasps answering :meth:`match`.
        """
        return list(
            self.match().evaluate(
                backend=ProbabilisticBackend(
                    model_registry or UniformPriorRegistry(),
                    number_of_samples=number_of_grasps,
                )
            )
        )

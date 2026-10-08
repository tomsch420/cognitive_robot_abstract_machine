"""
Finding the part of an object's shape that is its handle, so that the object's
annotation can be given a
:class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Handle` to be
grasped at.

Shapes are taken in the frame of the object's root body, its z-axis pointing up.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from functools import cached_property

import numpy as np
from numpy.typing import NDArray
import trimesh
from scipy.spatial import ConvexHull
from typing_extensions import Optional

from semantic_digital_twin.datastructures.definitions import Axis
from semantic_digital_twin.spatial_types import Point2

# %% finding handles


class HandleFinder(ABC):
    """
    Finds the part of a shape that is its handle.
    """

    @abstractmethod
    def find(self, shape: trimesh.Trimesh) -> Optional[trimesh.Trimesh]:
        """
        :param shape: The object's shape, in its root body's frame.
        :return: The part of ``shape`` that is its handle; ``None`` if it has none.
        """


@dataclass
class RoundOutline:
    """
    A circle in the horizontal plane.
    """

    center: Point2
    """
    The circle's center.
    """

    radius: float
    """
    The circle's radius.
    """

    @classmethod
    def fitted_to(cls, points: NDArray[np.float64]) -> RoundOutline:
        """
        :param points: At least three points in the horizontal plane, as rows of x
            and y.
        :return: The circle closest to the points, in the least squares sense.
        """
        coefficients, *_ = np.linalg.lstsq(
            np.column_stack([2 * points, np.ones(len(points))]),
            np.sum(points**2, axis=1),
            rcond=None,
        )
        x, y, offset = coefficients
        return cls(
            center=Point2(x, y),
            radius=float(np.sqrt(offset + x**2 + y**2)),
        )

    def distances(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """
        :param points: Points in the horizontal plane, as rows of x and y.
        :return: Each point's distance from the circle's center.
        """
        return np.linalg.norm(points - self.center.to_np(), axis=1)


@dataclass
class ProtrudingHandleFinder(HandleFinder):
    """
    Finds a handle that sticks out sideways from a round body, as a mug's or a pot's
    does.

    The body's outline, seen from above, is a circle; the handle is what lies outside of
    it.
    """

    clearance: float = 0.004
    """
    How far outside the round body a face has to lie to belong to the handle, in meters.
    """

    outline_tolerance: float = 0.002
    """
    How far outside the fitted circle a point of the outline may lie and still belong to
    the round body, in meters.
    """

    maximum_refits: int = 10
    """
    How often the circle is fitted again after leaving out the points outside of it.
    """

    def find(self, shape: trimesh.Trimesh) -> Optional[trimesh.Trimesh]:
        outline = self._round_body_outline(shape)
        outside = (
            outline.distances(shape.triangles_center[:, :2])
            > outline.radius + self.clearance
        )
        if not outside.any():
            return None
        return shape.submesh([np.flatnonzero(outside)], append=True)

    def _round_body_outline(self, shape: trimesh.Trimesh) -> RoundOutline:
        """
        Fit a circle to the outline of the shape seen from above, leaving out the points
        that stick out of it, until none do.

        :param shape: The object's shape.
        :return: The outline of its round body.
        """
        horizontal = shape.vertices[:, :2]
        points = horizontal[ConvexHull(horizontal).vertices]
        outline = RoundOutline.fitted_to(points)
        for _ in range(self.maximum_refits):
            inside = (
                outline.distances(points) <= outline.radius + self.outline_tolerance
            )
            if inside.all():
                return outline
            points = points[inside]
            outline = RoundOutline.fitted_to(points)
        return outline


@dataclass
class ElongatedShape:
    """
    A shape that is longer than it is wide, measured along its length and across its
    width at its vertices and the centers of its faces.
    """

    shape: trimesh.Trimesh
    """
    The shape.
    """

    length_axis: Axis
    """
    The axis the shape is longest along.
    """

    width_axis: Axis
    """
    The axis its width is measured along.
    """

    end_fraction: float = 0.15
    """
    How much of the shape's length, from each end, is compared to find the narrower end.
    """

    @cached_property
    def points(self) -> NDArray[np.float64]:
        """
        :return: The points the shape is measured at, as rows of x, y and z.
        """
        return np.vstack([self.shape.vertices, self.shape.triangles_center])

    @property
    def along(self) -> NDArray[np.float64]:
        """
        :return: Each point's distance from the end towards the negative length axis.
        """
        coordinates = self.points[:, self.length_axis]
        return coordinates - coordinates.min()

    @property
    def across(self) -> NDArray[np.float64]:
        """
        :return: Each point's coordinate across the shape.
        """
        return self.points[:, self.width_axis]

    @property
    def length(self) -> float:
        """
        :return: The shape's length.
        """
        return float(np.ptp(self.points[:, self.length_axis]))

    def narrower_end_is_positive(self) -> bool:
        """
        :return: Whether the end towards the positive length axis is the narrower one.
        """
        band = self.end_fraction * self.length
        negative_end = self.across[self.along <= band]
        positive_end = self.across[self.along >= self.length - band]
        return np.ptp(positive_end) < np.ptp(negative_end)


@dataclass
class NarrowEndHandleFinder(HandleFinder):
    """
    Finds the handle of an elongated object lying flat, such as a piece of cutlery or a
    tool: the stretch from its narrower end up to where it widens.
    """

    widening: float = 1.8
    """
    How many times wider than at its narrow end the object has to get for the handle to
    end there.
    """

    slice_length: float = 0.005
    """
    The length of the slices the object's width is measured in, in meters.
    """

    end_fraction: float = 0.15
    """
    How much of the object's length, from each end, is compared to find the narrower
    end.
    """

    def find(self, shape: trimesh.Trimesh) -> Optional[trimesh.Trimesh]:
        lowest, highest = shape.bounds
        extents = highest - lowest
        length_axis = Axis.X if extents[Axis.X] >= extents[Axis.Y] else Axis.Y
        elongated = ElongatedShape(
            shape=shape,
            length_axis=length_axis,
            width_axis=Axis.Y if length_axis == Axis.X else Axis.X,
            end_fraction=self.end_fraction,
        )
        along = elongated.along
        face_along = shape.triangles_center[:, length_axis] - lowest[length_axis]
        if elongated.narrower_end_is_positive():
            along = elongated.length - along
            face_along = elongated.length - face_along
        handle_length = self._handle_length(elongated.across, along)
        if handle_length is None:
            return None
        return shape.submesh([np.flatnonzero(face_along < handle_length)], append=True)

    def _handle_length(
        self, across: NDArray[np.float64], along: NDArray[np.float64]
    ) -> Optional[float]:
        """
        :param across: The points' coordinates across the object.
        :param along: The points' distances from the narrow end.
        :return: How far from the narrow end the object first gets :attr:`widening`
            times as wide as at that end; ``None`` if it never does.
        """
        widths = []
        for start in np.arange(0.0, along.max(), self.slice_length):
            in_slice = across[(along >= start) & (along < start + self.slice_length)]
            if len(in_slice) > 1:
                widths.append((start, float(np.ptp(in_slice))))
        end_width = max(widths[0][1], 1e-4)
        for start, width in widths:
            if width > self.widening * end_width:
                return start
        return None

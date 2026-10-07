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

import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from typing_extensions import Optional

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

    center: np.ndarray
    """
    The circle's center, as x and y.
    """

    radius: float
    """
    The circle's radius.
    """

    @classmethod
    def fitted_to(cls, points: np.ndarray) -> RoundOutline:
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
        center = coefficients[:2]
        return cls(
            center=center,
            radius=float(np.sqrt(coefficients[2] + center @ center)),
        )

    def distances(self, points: np.ndarray) -> np.ndarray:
        """
        :param points: Points in the horizontal plane, as rows of x and y.
        :return: Each point's distance from the circle's center.
        """
        return np.linalg.norm(points - self.center, axis=1)


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
        length_axis = 0 if extents[0] >= extents[1] else 1
        width_axis = 1 - length_axis
        length = float(extents[length_axis])
        points = np.vstack([shape.vertices, shape.triangles_center])
        across = points[:, width_axis]
        along = points[:, length_axis] - lowest[length_axis]
        face_along = shape.triangles_center[:, length_axis] - lowest[length_axis]
        if self._narrower_end_is_far(across, along, length):
            along = length - along
            face_along = length - face_along
        handle_length = self._handle_length(across, along)
        if handle_length is None:
            return None
        return shape.submesh([np.flatnonzero(face_along < handle_length)], append=True)

    def _narrower_end_is_far(
        self, across: np.ndarray, along: np.ndarray, length: float
    ) -> bool:
        """
        :param across: The points' coordinates across the object.
        :param along: The points' distances from the object's near end.
        :param length: The object's length.
        :return: Whether the far end is the narrower one.
        """
        band = self.end_fraction * length
        near = across[along <= band]
        far = across[along >= length - band]
        return np.ptp(far) < np.ptp(near)

    def _handle_length(self, across: np.ndarray, along: np.ndarray) -> Optional[float]:
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

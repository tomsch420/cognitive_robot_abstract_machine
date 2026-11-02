"""
Frames, rays and bounding boxes in the world's root frame, as the tasks reason about
where things are.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from semantic_digital_twin.spatial_types.spatial_types import (
    HomogeneousTransformationMatrix,
)
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import ActiveConnection1DOF
from semantic_digital_twin.world_description.geometry import CoordinateBounds
from semantic_digital_twin.world_description.world_entity import (
    Body,
    KinematicStructureEntity,
)

# %% frames


@dataclass
class ApproachFrame:
    """
    A frame whose x-axis points along the way a gripper comes in and whose y-axis along
    the way its fingers close.
    """

    position: np.ndarray
    """
    The origin of the frame.
    """

    approach: np.ndarray
    """
    The direction the x-axis points in.
    """

    closing: np.ndarray
    """
    The direction the y-axis points in, square to :attr:`approach`.
    """

    def in_reference_frame(
        self, reference_frame: KinematicStructureEntity
    ) -> HomogeneousTransformationMatrix:
        """
        :param reference_frame: The frame :attr:`position`, :attr:`approach` and
            :attr:`closing` are given in.
        :return: The frame.
        """
        x_axis = np.asarray(self.approach, dtype=float)
        y_axis = np.asarray(self.closing, dtype=float)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        matrix = np.eye(4)
        matrix[:3, 0] = x_axis
        matrix[:3, 1] = y_axis
        matrix[:3, 2] = np.cross(x_axis, y_axis)
        matrix[:3, 3] = self.position
        return HomogeneousTransformationMatrix(matrix, reference_frame=reference_frame)


# %% rays


@dataclass
class Ray:
    """
    A line through a point along a direction.
    """

    point: np.ndarray
    """
    A point the ray passes through.
    """

    direction: np.ndarray
    """
    The direction it runs in.
    """

    def entry_into(
        self, bounds: CoordinateBounds, parallel_tolerance: float = 1e-9
    ) -> np.ndarray:
        """
        :param bounds: A box that holds :attr:`point`.
        :param parallel_tolerance: How small a component of :attr:`direction` counts
            as running parallel to the box's sides along that axis.
        :return: Where the ray enters the box before reaching :attr:`point`.
        """
        with np.errstate(divide="ignore"):
            steps = np.where(
                self.direction > parallel_tolerance,
                (self.point - bounds.lower) / self.direction,
                np.where(
                    self.direction < -parallel_tolerance,
                    (self.point - bounds.upper) / self.direction,
                    np.inf,
                ),
            )
        return self.point - self.direction * float(np.min(steps))


# %% bodies


@dataclass
class BodyBounds:
    """
    The boxes bounding the collision geometry of the world's bodies, axis-aligned in the
    world's root frame.
    """

    world: World
    """
    The world the bodies belong to.
    """

    def of_body(self, body: Body) -> CoordinateBounds:
        """
        :param body: A body with collision geometry.
        :return: The box bounding its own collision geometry.
        """
        return (
            body.collision.as_bounding_box_collection_in_frame(self.world.root)
            .bounding_box()
            .to_array_bounds()
        )

    def center_of(self, body: Body) -> np.ndarray:
        """
        :param body: A body with collision geometry.
        :return: The centre of the box bounding its own collision geometry.
        """
        bounds = self.of_body(body)
        return (bounds.lower + bounds.upper) / 2

    def of_branch(self, body: Body, rigidly_attached: bool = False) -> CoordinateBounds:
        """
        :param body: A body with collision geometry, or with collision geometry below
            it.
        :param rigidly_attached: Whether only what is fixed to the body is bounded,
            leaving out what hangs from a joint below it.
        :return: The box bounding the body and everything below it.
        """
        below = self.world.compute_descendent_child_kinematic_structure_entities(body)
        if rigidly_attached:
            joint = self.joint_above(body)
            below = [part for part in below if self.joint_above(part) is joint]
        boxes = [
            self.of_body(part)
            for part in [body, *below]
            if isinstance(part, Body) and part.has_collision()
        ]
        return CoordinateBounds(
            np.min([box.lower for box in boxes], axis=0),
            np.max([box.upper for box in boxes], axis=0),
        )

    def distance_to_nearest_shape(self, body: Body, point: np.ndarray) -> float:
        """
        :param body: A body with collision geometry.
        :param point: A point in the world's root frame.
        :return: How far the point lies from the nearest of the boxes bounding the
            body's own collision shapes one by one; 0 inside one of them.
        """
        return min(
            float(np.linalg.norm(point - np.clip(point, bounds.lower, bounds.upper)))
            for bounds in (
                box.to_array_bounds()
                for box in body.collision.as_bounding_box_collection_in_frame(
                    self.world.root
                ).bounding_boxes
            )
        )

    def distance_to(self, body: Body, point: np.ndarray) -> float:
        """
        :param body: A body with collision geometry, or with collision geometry below
            it.
        :param point: A point in the world's root frame.
        :return: How far the point lies from the box bounding the body and everything
            below it; 0 inside the box.
        """
        box = self.of_branch(body)
        return float(np.linalg.norm(point - np.clip(point, box.lower, box.upper)))

    @staticmethod
    def joint_above(body: KinematicStructureEntity) -> ActiveConnection1DOF | None:
        """
        :param body: A body of the world.
        :return: The nearest joint of one degree of freedom above the body, if any.
        """
        entity = body
        while entity.parent_connection is not None:
            if isinstance(entity.parent_connection, ActiveConnection1DOF):
                return entity.parent_connection
            entity = entity.parent_connection.parent
        return None

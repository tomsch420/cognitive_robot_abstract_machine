"""
The rim of an open container, the upper edge of its wall, which a gripper pinches from
above.

Shapes are taken in the frame of the container's root body, its z-axis pointing up.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import trimesh
from typing_extensions import Optional

from krrood.entity_query_language.query.match import Match
from semantic_digital_twin.datastructures.definitions import Axis
from semantic_digital_twin.grasping.surface_grasp import any_surface_grasp


@dataclass
class Rim:
    """
    The upper edge of an open container's wall, gripped a little below the container's
    highest point.
    """

    container_height: float
    """
    How tall the container is, in meters.
    """

    wall_thickness: float
    """
    How thick the wall is where it is gripped, in meters.
    """

    grip_depth: float
    """
    How far below the container's highest point the wall is gripped, in meters.
    """

    @classmethod
    def measured_on(
        cls, shape: trimesh.Trimesh, grip_depth: float, probe_count: int = 12
    ) -> Optional[Rim]:
        """
        Cast rays outward from the middle of the container, ``grip_depth`` below its
        highest point, and take the distance between the first two surfaces each ray
        passes, the inside and the outside of the wall, as the wall's thickness.

        :param shape: The container's shape.
        :param grip_depth: How far below the container's highest point the wall is
            gripped, in meters.
        :param probe_count: In how many directions around the container the wall is
            measured.
        :return: The rim, its wall as thick as the median over the directions that hit
            the wall; ``None`` if no direction does.
        """
        yaws = np.linspace(0, 2 * np.pi, probe_count, endpoint=False)
        directions = np.column_stack([np.cos(yaws), np.sin(yaws), np.zeros(len(yaws))])
        lowest, highest = shape.bounds
        axis_point = (lowest + highest) / 2
        axis_point[Axis.Z] = highest[Axis.Z] - grip_depth
        locations, ray_indices, _ = shape.ray.intersects_location(
            ray_origins=np.tile(axis_point, (len(yaws), 1)), ray_directions=directions
        )
        thicknesses = []
        for index in range(len(yaws)):
            distances = np.sort(
                np.linalg.norm(
                    locations[ray_indices == index][:, :2] - axis_point[:2], axis=1
                )
            )
            if len(distances) > 1:
                thicknesses.append(float(distances[1] - distances[0]))
        if not thicknesses:
            return None
        return cls(
            container_height=float(highest[Axis.Z] - lowest[Axis.Z]),
            wall_thickness=float(np.median(thicknesses)),
            grip_depth=grip_depth,
        )

    def grasp_statement(self, require_lifting: bool = False) -> Match:
        """
        :param require_lifting: Whether to ask only for grasps that lift the container.
        :return: The statement of grasps in a band :attr:`grip_depth` below the rim, all
            around, closing on the middle half of the wall's thickness, from above or
            tilted a little towards the wall.
        """
        grip_height = 1.0 - self.grip_depth / self.container_height
        band = self.grip_depth / (2 * self.container_height)
        grasp = any_surface_grasp(require_lifting)
        return grasp.where(
            grasp.height >= grip_height - band,
            grasp.height < min(grip_height + band, 1.0),
            grasp.depth >= 0.25 * self.wall_thickness,
            grasp.depth < 0.75 * self.wall_thickness,
            grasp.azimuth >= 0.0,
            grasp.azimuth < 2 * math.pi,
            grasp.pitch >= 0.0,
            grasp.pitch < 0.5,
            grasp.roll >= -math.pi / 8,
            grasp.roll < math.pi / 8,
        )

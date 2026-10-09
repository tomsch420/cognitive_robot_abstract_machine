"""
Finding the piece of an open container's shape that is its rim, so that it can be split
off into a :class:`~semantic_digital_twin.semantic_annotations.semantic_annotations.Rim`
of its own by :class:`~semantic_digital_twin.pipeline.part_splitting.SplitPartFromShape`.

Shapes are taken in the frame of the container's root body, its z-axis pointing up.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh
from typing_extensions import Optional

from semantic_digital_twin.datastructures.definitions import Axis
from semantic_digital_twin.pipeline.part_splitting import PartFinder, ShapeSplit


@dataclass
class RimFinder(PartFinder):
    """
    Finds the rim of an open container: the band of its shape just below its highest
    point, cut off horizontally.
    """

    depth: float = 0.02
    """
    How far below the container's highest point the rim reaches, in meters.
    """

    def split(self, shape: trimesh.Trimesh) -> Optional[ShapeSplit]:
        if shape.is_empty:
            return None
        up = np.zeros(3)
        up[Axis.Z] = 1.0
        lowest_rim_point = shape.bounds[1] - up * self.depth
        return ShapeSplit.by_plane(shape, lowest_rim_point, up)

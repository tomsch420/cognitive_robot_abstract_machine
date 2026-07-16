"""
Semantic annotations for the objects that appear in the ``icub_montessori_no_hands``
episode: the shape-sorting board, its shape-holes, and the loose shapes that are
dropped through them.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from typing_extensions import Optional

from krrood.ormatic.utils import classproperty
from semantic_digital_twin.semantic_annotations.mixins import (
    HasApertures,
    HasCaseAsRootBody,
    HasDrawers,
    HasRootBody,
)
from semantic_digital_twin.semantic_annotations.semantic_annotations import Aperture
from semantic_digital_twin.spatial_types import Vector3


class MontessoriShapeCategory(StrEnum):
    """
    The geometric shape of a loose Montessori piece and of the single hole in the
    ShapeSortingBoard it is meant to be dropped through.
    """

    CUBE = "cube"
    CYLINDER = "cylinder"
    DISK = "disk"
    SPHERE = "sphere"
    TRIANGULAR_PRISM = "triangular_prism"
    RECTANGULAR_PRISM = "rectangular_prism"


@dataclass(eq=False)
class MontessoriShape(HasRootBody):
    """
    A loose Montessori piece that a child (or robot) drops through the matching
    :class:`ShapeSortingHole` of a :class:`ShapeSortingBoard`.
    """

    shape_category: MontessoriShapeCategory = field(kw_only=True)
    """
    The geometric shape of this piece, matched against
    :attr:`ShapeSortingHole.shape_category` to decide which hole it fits through.
    """


@dataclass(eq=False)
class ShapeSortingHole(Aperture):
    """
    A hole cut into the lid of a :class:`ShapeSortingBoard`, shaped after a single
    :class:`MontessoriShapeCategory`; only a :class:`MontessoriShape` of the matching
    category passes through it.
    """

    shape_category: Optional[MontessoriShapeCategory] = field(
        kw_only=True, default=None
    )
    """
    The geometric shape of this hole, matched against
    :attr:`MontessoriShape.shape_category` to decide which pieces fit through it.
    """


@dataclass(eq=False)
class ShapeSortingBoard(HasCaseAsRootBody, HasDrawers, HasApertures):
    """
    The Montessori shape-sorting board: a wooden case whose lid has one
    :class:`ShapeSortingHole` per :class:`MontessoriShapeCategory` and whose body
    houses drawers for storing the shapes when they are not on the board.
    """

    @classproperty
    def hole_direction(self) -> Vector3:
        return Vector3.Z()

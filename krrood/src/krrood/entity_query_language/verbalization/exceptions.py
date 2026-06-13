"""
Custom exceptions raised by the EQL verbalization subsystem.

Each is a :class:`~krrood.exceptions.DataclassException` — a dataclass exception that carries the
structured cause as fields and builds its human-readable ``message`` in ``__post_init__`` — so the
failure data is inspectable (not only a formatted string), consistent with the rest of krrood.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from typing_extensions import List

from krrood.exceptions import DataclassException

if TYPE_CHECKING:
    from krrood.entity_query_language.core.base_expressions import SymbolicExpression
    from krrood.entity_query_language.verbalization.fragments.base import Fragment
    from krrood.entity_query_language.verbalization.grammar.restriction import Placement


@dataclass
class UnverbalizableExpressionError(DataclassException):
    """
    No grammar rule covers an EQL construct — a coverage gap, surfaced as an error rather than
    silently degrading the node to its class name.
    """

    node: "SymbolicExpression"
    """The EQL expression no grammar rule matched."""

    def __post_init__(self):
        self.message = (
            f"No verbalization rule for {type(self.node).__name__!r} "
            f"(name={getattr(self.node, '_name_', None)!r}); "
            "add a PhraseRule in grammar/english.py."
        )
        super().__post_init__()


@dataclass
class UnloweredFragmentError(DataclassException):
    """
    A fragment kind with no fold handler reached a renderer — a realisation pass was skipped,
    leaving an un-lowered ``NounPhrase`` or ``PossessiveChain`` in the tree.
    """

    fragment: "Fragment"
    """The un-lowered fragment that reached the renderer."""

    def __post_init__(self):
        self.message = (
            f"fold_fragment received a {type(self.fragment).__name__}; "
            "NounPhrase / PossessiveChain nodes must be lowered by the realisation "
            "passes (realize_tree) before a renderer folds the tree."
        )
        super().__post_init__()


@dataclass
class UnplacedRestrictionError(DataclassException):
    """
    A restriction rule declared a placement that no ``RestrictionFragments`` slot surfaces.
    """

    placements: List["Placement"]
    """The leftover placement(s) the assembler does not surface."""

    def __post_init__(self):
        self.message = (
            "Restriction placement(s) with no RestrictionFragments slot: "
            f"{[placement.name for placement in self.placements]}."
        )
        super().__post_init__()

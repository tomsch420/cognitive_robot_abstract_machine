"""
Utilities for walking MappedVariable chains and building path parts.

These are pure utilities shared by multiple verbalizer subsystems.  They must
not import from the subsystem files (``verbalizer.py``, ``context.py``) to avoid
circular dependencies.  Imports from EQL core and from the ``fragments/`` layer
are safe because those are lower in the dependency graph.
"""

from __future__ import annotations

import datetime as _dt
from dataclasses import dataclass

from typing_extensions import List, Optional, Tuple

from krrood.entity_query_language.core.mapped_variable import (
    Attribute,
    Call,
    FlatVariable,
    Index,
    MappedVariable,
)
from krrood.entity_query_language.core.variable import Literal, Variable
from krrood.entity_query_language.verbalization.fragments.source_ref import SourceRef


def walk_chain(expression) -> tuple[list, object]:
    """
    Walk a :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
    chain outward-first and return ``(chain, root)``.

    The chain is reversed so index 0 is the outermost (innermost to the call
    site) node; the root is the first non-MappedVariable child.

    Example: for ``robot.arm.joint`` the chain is
    ``[Attribute('joint'), Attribute('arm')]`` and root is the ``robot`` Variable.

    :param expression: Any expression; non-MappedVariable expressions return an
        empty chain with *expression* as the root.
    :return: Tuple ``(chain, root)`` where *chain* is the core
        :attr:`~krrood.entity_query_language.core.mapped_variable.MappedVariable._access_path_`
        (root-adjacent first, terminal last) and *root* is the chain base.
    :rtype: tuple[list, object]
    """
    if isinstance(expression, MappedVariable):
        return list(expression._access_path_), expression._chain_root_
    return [], expression


def is_temporal(expression) -> bool:
    """
    Return ``True`` when *expression* denotes a :class:`datetime.datetime` value or variable.

    Used by comparator verbalization to select temporal operator phrases
    (*"is before"* / *"is after"*) instead of relational ones.  Inspects the
    expression's ``_type_`` (or a :class:`~krrood.entity_query_language.core.variable.Literal`'s
    value), so it is a pure structural/type check with no verbalization state.

    :param expression: Any EQL expression.
    :return: ``True`` when the expression is datetime-typed.
    :rtype: bool
    """
    if isinstance(expression, Literal):
        return isinstance(expression._value_, _dt.datetime)
    if isinstance(expression, Variable):
        return getattr(expression, "_type_", None) is _dt.datetime
    if isinstance(expression, MappedVariable):
        chain, _ = walk_chain(expression)
        return bool(chain) and getattr(chain[-1], "_type_", None) is _dt.datetime
    return False


def chain_root(expression) -> object:
    """
    Return the non-:class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
    root of *expression* without building the full chain list.

    Faster than :func:`walk_chain` when only the root is needed.

    :param expression: Any expression.
    :return: The deepest non-MappedVariable node in the chain, or *expression* itself
        when it is not a MappedVariable.
    :rtype: object
    """
    return (
        expression._chain_root_
        if isinstance(expression, MappedVariable)
        else expression
    )


def build_path_parts(chain: list) -> list[tuple[str, Optional[SourceRef]]]:
    """
    Convert a walked chain (from :func:`walk_chain`) into ``(display_name, SourceRef | None)`` pairs.

    Merging rules:

    * Consecutive ``Attribute → Index`` pairs are merged into ``"attr[key]"`` with ``ref=None``
      (composite indexed access has no clean single-symbol anchor).
    * Standalone :class:`~krrood.entity_query_language.core.mapped_variable.Index` nodes
      appear as ``"[key]"`` with ``ref=None``.
    * :class:`~krrood.entity_query_language.core.mapped_variable.Call` nodes appear as ``"()"``
      with ``ref=None``.
    * :class:`~krrood.entity_query_language.core.mapped_variable.FlatVariable` nodes are skipped.

    :param chain: Outermost-first chain list from :func:`walk_chain`.
    :type chain: list
    :return: Ordered list of ``(display_name, SourceRef | None)`` pairs,
        outermost attribute first.
    :rtype: list[tuple[str, SourceRef | None]]
    """
    parts: list[tuple[str, Optional[SourceRef]]] = []
    i = 0
    while i < len(chain):
        node = chain[i]
        if isinstance(node, Attribute):
            name = node._attribute_name_
            owner = node._owner_class_
            ref: Optional[SourceRef] = SourceRef.for_attribute(owner, name)
            while i + 1 < len(chain) and isinstance(chain[i + 1], Index):
                i += 1
                name += f"[{repr(chain[i]._key_)}]"
                ref = None  # composite indexed access has no clean single-line anchor
            parts.append((name, ref))
        elif isinstance(node, Index):
            parts.append((f"[{repr(node._key_)}]", None))
        elif isinstance(node, Call):
            parts.append(("()", None))
        elif isinstance(node, FlatVariable):
            pass
        i += 1
    return parts


@dataclass(frozen=True)
class ChainAnalysis:
    """A MappedVariable chain analysed **once**.

    The chain rendering needs the walked chain, its root, the display path-parts, and whether
    it ends in a boolean attribute — previously each was recomputed (``walk_chain`` /
    ``build_path_parts`` / ``is_bool_attr_chain``) at separate call sites on the same
    expression.  :meth:`of` computes them together so the assembler can branch off one value.
    """

    chain: List
    """The access path, root-adjacent first (from :func:`walk_chain`)."""

    root: object
    """The chain root (first non-MappedVariable node)."""

    parts: List[Tuple[str, Optional[SourceRef]]]
    """The display path-parts (from :func:`build_path_parts`)."""

    is_bool_terminal: bool
    """``True`` when the chain ends in a ``bool``-typed :class:`Attribute` (predicative form)."""

    @classmethod
    def of(cls, expression) -> "ChainAnalysis":
        """Analyse *expression* (a MappedVariable chain, or any root expression)."""
        chain, root = walk_chain(expression)
        is_bool_terminal = (
            bool(chain)
            and isinstance(chain[-1], Attribute)
            and chain[-1]._type_ is bool
        )
        return cls(
            chain=chain,
            root=root,
            parts=build_path_parts(chain),
            is_bool_terminal=is_bool_terminal,
        )

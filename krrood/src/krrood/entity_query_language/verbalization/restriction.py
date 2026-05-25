"""
Subject restriction clauses — turning a subject's ``WHERE`` condition into a
post-nominal *"whose <attr> is …"* modifier plus a residual *"such that …"* clause.

The "is this conjunct a groupable subject predicate, and how does it render?" decision
is **declarative**: a small :class:`RestrictionRule` hierarchy, dispatched first-match-wins
by specificity (mirroring the main
:class:`~krrood.entity_query_language.verbalization.rule_engine.RuleEngine`).  A folded
conjunct that **no** rule matches becomes residual — groupability lives in the rule
preconditions, not in branching glue.

:class:`RestrictionClauseBuilder` is the thin orchestrator used by
:class:`~krrood.entity_query_language.verbalization.entity_verbalizer.EntityVerbalizer`.
"""

from __future__ import annotations

import operator
from abc import ABC, abstractmethod
from typing import List, Optional, TYPE_CHECKING

from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import AND
from krrood.entity_query_language.verbalization.chain_utils import walk_chain
from krrood.entity_query_language.verbalization.fragments.base import (
    oxford_and,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.range_fold import (
    build_between,
    fold_range_pairs,
    RangeFold,
)
from krrood.entity_query_language.verbalization.subquery import is_calculation_value
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Conjunctions,
    Keywords,
    Operators,
)

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


# ── Chain helpers ──────────────────────────────────────────────────────────────


def _single_hop_attr(expr, subject_var):
    """Return the :class:`Attribute` node when *expr* is exactly ``subject_var.<attr>``, else ``None``."""
    from krrood.entity_query_language.core.mapped_variable import (
        Attribute,
        MappedVariable,
    )
    from krrood.entity_query_language.core.variable import Variable

    if subject_var is None or not isinstance(expr, MappedVariable):
        return None
    chain, root = walk_chain(expr)
    if not (isinstance(root, Variable) and root._id_ == subject_var._id_):
        return None
    if len(chain) != 1 or not isinstance(chain[0], Attribute):
        return None
    return chain[0]


def _references(expr, subject_var) -> bool:
    """Return ``True`` when *expr* mentions *subject_var* (so it is not a clean RHS value)."""
    try:
        return any(
            getattr(v, "_id_", None) == subject_var._id_
            for v in expr._unique_variables_
        )
    except AttributeError:
        from krrood.entity_query_language.verbalization.chain_utils import chain_root

        return chain_root(expr) is subject_var


def _predicate_op_frag(
    comparator: Comparator, ctx: "VerbalizationContext", delegate
) -> VerbFragment:
    """Operator fragment for a bare-attribute predicate (standard form, keeps the copula)."""
    if comparator.operation in (operator.eq, operator.ne) and (
        is_calculation_value(comparator.left) or is_calculation_value(comparator.right)
    ):
        return Operators.CALC_EQ.select(
            negated=comparator.operation is operator.ne, compact=False
        ).as_fragment()
    is_temporal = delegate._chain.is_temporal(
        comparator.left
    ) or delegate._chain.is_temporal(comparator.right)
    try:
        return (
            Operators.from_callable(comparator.operation)
            .select(compact=False, temporal=is_temporal)
            .as_fragment()
        )
    except KeyError:
        return RoleFragment.for_operator(comparator._name_)


# ── Restriction rule hierarchy ──────────────────────────────────────────────────


class RestrictionRule(ABC):
    """
    Declarative rule that recognises a folded conjunct as a groupable subject
    restriction and renders it as a bare post-nominal predicate (no root / pronoun;
    the enclosing *"whose"* supplies the subject).

    A conjunct matched by no rule is residual and stays in a *"such that …"* clause.
    """

    @classmethod
    @abstractmethod
    def applies(cls, item, subject_var, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` when *item* is a groupable restriction on *subject_var*."""

    @classmethod
    @abstractmethod
    def render(
        cls, item, subject_var, ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """Render *item* as a bare predicate fragment (e.g. *"booking_date is between X and Y"*)."""


class RangeRestrictionRule(RestrictionRule):
    """Matches a :class:`RangeFold` on a single-hop subject attribute → *"<attr> is between lo and hi"*."""

    @classmethod
    def applies(cls, item, subject_var, ctx: "VerbalizationContext") -> bool:
        return (
            isinstance(item, RangeFold)
            and _single_hop_attr(item.chain_expr, subject_var) is not None
        )

    @classmethod
    def render(
        cls, item, subject_var, ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        attr = _single_hop_attr(item.chain_expr, subject_var)
        left = RoleFragment.for_attribute(attr._owner_class_, attr._attribute_name_)
        return build_between(
            left,
            delegate.build(item.lo_expr, ctx),
            delegate.build(item.hi_expr, ctx),
            compact=False,
        )


class AttributePredicateRestrictionRule(RestrictionRule):
    """
    Matches a :class:`~krrood.entity_query_language.operators.comparator.Comparator`
    that is a single-hop, non-boolean subject-attribute predicate whose RHS does not
    reference the subject → *"<attr> is greater than 100"* / *"<attr> is equal to <calc>"*.
    """

    @classmethod
    def applies(cls, item, subject_var, ctx: "VerbalizationContext") -> bool:
        if not isinstance(item, Comparator):
            return False
        attr = _single_hop_attr(item.left, subject_var)
        if attr is None or attr._type_ is bool:
            return False
        return not _references(item.right, subject_var)

    @classmethod
    def render(
        cls, item, subject_var, ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        attr = _single_hop_attr(item.left, subject_var)
        attr_frag = RoleFragment.for_attribute(
            attr._owner_class_, attr._attribute_name_
        )
        op_frag = _predicate_op_frag(item, ctx, delegate)
        return PhraseFragment(
            parts=[attr_frag, op_frag, delegate.build(item.right, ctx)], separator=" "
        )


RESTRICTION_RULES: List[type] = [
    RangeRestrictionRule,
    AttributePredicateRestrictionRule,
]


# ── Builder ─────────────────────────────────────────────────────────────────────


class RestrictionClauseBuilder:
    """
    Partition a subject's ``WHERE`` condition into a *"whose <grouped>"* modifier and a
    residual condition fragment.

    :param delegate: The parent verbalizer for recursive sub-expression rendering.
    """

    def __init__(self, delegate: "EQLVerbalizer") -> None:
        self._d = delegate

    def build(
        self, subject_var, condition, ctx: "VerbalizationContext"
    ) -> "tuple[Optional[VerbFragment], Optional[VerbFragment]]":
        """
        :param subject_var: The subject the conditions restrict (its single-hop attribute
            predicates become *"whose"* modifiers); may be ``None`` (nothing groups).
        :param condition: The ``WHERE`` condition expression.
        :param ctx: Shared verbalization state. Residual conditions are rendered through the
            main engine, so push *subject_var* as the coreference subject **before** calling
            this when pronouns are wanted in the residual.
        :returns: ``(whose_modifier | None, residual_condition | None)``.  ``whose_modifier``
            already includes the ``whose`` keyword; ``residual_condition`` does **not** include
            a ``such that`` / ``where`` keyword (the caller adds the appropriate one).
        :rtype: tuple
        """
        items = fold_range_pairs(ctx.flatten_same_type(condition, AND))
        grouped: List[VerbFragment] = []
        residual: List = []
        for item in items:
            rule = self._match(item, subject_var, ctx)
            if rule is None:
                residual.append(item)
            else:
                grouped.append(rule.render(item, subject_var, ctx, self._d))

        whose_frag = None
        if grouped:
            whose_frag = PhraseFragment(
                parts=[
                    Keywords.WHOSE.as_fragment(),
                    oxford_and(grouped, Conjunctions.AND.as_fragment()),
                ],
                separator=" ",
            )
        residual_frag = self._render_residual(residual, ctx) if residual else None
        return whose_frag, residual_frag

    @staticmethod
    def _match(item, subject_var, ctx: "VerbalizationContext") -> Optional[type]:
        for rule in RESTRICTION_RULES:
            if rule.applies(item, subject_var, ctx):
                return rule
        return None

    def _render_residual(
        self, items: List, ctx: "VerbalizationContext"
    ) -> VerbFragment:
        parts: List[VerbFragment] = []
        for item in items:
            if isinstance(item, RangeFold):
                parts.append(
                    build_between(
                        self._d.build(item.chain_expr, ctx),
                        self._d.build(item.lo_expr, ctx),
                        self._d.build(item.hi_expr, ctx),
                        compact=ctx.compact_predicates,
                    )
                )
            else:
                parts.append(self._d.build(item, ctx))
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())

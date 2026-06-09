"""
Subject restriction systems — recognising a subject's ``WHERE`` conjunct as a
post-nominal *"whose <attr> is …"* modifier (vs. a residual *"such that …"* clause),
and resolving **which** variable a selection restricts.

Both decisions are **declarative**: small rule hierarchies whose preconditions carry
the analysis, dispatched first-by-specificity through the *same*
:func:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.most_specific`
primitive that selects :class:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.PhraseRule`
— so there is no bespoke dispatch loop.  Matching is **pure analysis** (used by
:class:`~krrood.entity_query_language.verbalization.grammar.planning.query.QueryPlanner`);
each rule's :meth:`render` is the realisation half (used by
:class:`~krrood.entity_query_language.verbalization.grammar.assembly.query.QueryAssembler`).

Reference: Dale & Reiter (1995) — referring expressions / post-nominal modification.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING
from typing_extensions import ClassVar, List, Optional, Type

from krrood.entity_query_language.core.mapped_variable import Attribute, MappedVariable
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.operators.aggregators import Aggregator
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.verbalization.chain_utils import (
    chain_root,
    walk_chain,
)
from krrood.entity_query_language.verbalization.fragments.base import (
    PhraseFragment,
    RoleFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.grammar.phrase_rule import (
    Ctx,
    most_specific,
)
from krrood.entity_query_language.verbalization.microplanning.coordination import (
    build_between,
    RangeFold,
)
from krrood.entity_query_language.verbalization.operator_phrase import (
    comparator_operator,
)
from krrood.entity_query_language.verbalization.subquery import aggregation_source_root

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext


# ── chain helpers ────────────────────────────────────────────────────────────


def _single_hop_attr(expression, subject_variable):
    """The :class:`Attribute` node when *expression* is exactly ``subject_variable.<attr>``, else ``None``."""
    if subject_variable is None or not isinstance(expression, MappedVariable):
        return None
    chain, root = walk_chain(expression)
    if not (isinstance(root, Variable) and root._id_ == subject_variable._id_):
        return None
    if len(chain) != 1 or not isinstance(chain[0], Attribute):
        return None
    return chain[0]


def _references(expression, subject_variable) -> bool:
    """``True`` when *expression* mentions *subject_variable* (so it is not a clean RHS value)."""
    try:
        return any(
            getattr(variable, "_id_", None) == subject_variable._id_
            for variable in expression._unique_variables_
        )
    except AttributeError:
        return chain_root(expression) is subject_variable


# ── restriction rules (groupable conjunct → bare predicate) ──────────────────


class RestrictionRule(ABC):
    """
    Recognise a folded conjunct as a groupable subject restriction and render it as a
    bare post-nominal predicate (the enclosing *"whose"* supplies the subject).  A
    conjunct matched by no rule is residual and stays in a *"such that …"* clause.
    """

    priority: ClassVar[int] = 0
    """Tiebreak when several rules apply (higher wins); mirrors ``PhraseRule.tiebreak``."""

    @classmethod
    @abstractmethod
    def applies(cls, item, subject_variable, context: "VerbalizationContext") -> bool:
        """Return ``True`` when *item* is a groupable restriction on *subject_variable*."""

    @classmethod
    @abstractmethod
    def render(cls, item, subject_variable, ctx: Ctx) -> VerbFragment:
        """Render *item* as a bare predicate fragment (recurses via ``ctx.child``)."""


class RangeRestrictionRule(RestrictionRule):
    """A :class:`RangeFold` on a single-hop subject attribute → *"<attr> is between lo and hi"*."""

    @classmethod
    def applies(cls, item, subject_variable, context: "VerbalizationContext") -> bool:
        return (
            isinstance(item, RangeFold)
            and _single_hop_attr(item.chain_expression, subject_variable) is not None
        )

    @classmethod
    def render(cls, item, subject_variable, ctx: Ctx) -> VerbFragment:
        attr = _single_hop_attr(item.chain_expression, subject_variable)
        left = RoleFragment.for_attribute(attr._owner_class_, attr._attribute_name_)
        return build_between(
            left,
            ctx.child(item.lower_expression),
            ctx.child(item.upper_expression),
            compact=False,
        )


class AttributePredicateRestrictionRule(RestrictionRule):
    """
    A single-hop, non-boolean subject-attribute :class:`Comparator` whose RHS does not
    reference the subject → *"<attr> is greater than 100"* / *"<attr> is equal to <calc>"*.
    """

    @classmethod
    def applies(cls, item, subject_variable, context: "VerbalizationContext") -> bool:
        if not isinstance(item, Comparator):
            return False
        attr = _single_hop_attr(item.left, subject_variable)
        if attr is None or attr._type_ is bool:
            return False
        return not _references(item.right, subject_variable)

    @classmethod
    def render(cls, item, subject_variable, ctx: Ctx) -> VerbFragment:
        attr = _single_hop_attr(item.left, subject_variable)
        attribute_fragment = RoleFragment.for_attribute(
            attr._owner_class_, attr._attribute_name_
        )
        operator_fragment = comparator_operator(item, ctx.context, compact=False)
        return PhraseFragment(
            parts=[attribute_fragment, operator_fragment, ctx.child(item.right)]
        )


RESTRICTION_RULES: List[Type[RestrictionRule]] = [
    RangeRestrictionRule,
    AttributePredicateRestrictionRule,
]


def match_restriction(
    item, subject_variable, context: "VerbalizationContext"
) -> Optional[Type[RestrictionRule]]:
    """The most-specific applicable :class:`RestrictionRule` for *item*, or ``None`` (residual).

    Pure analysis — no fragment is built; the planner uses this to partition conjuncts.
    """
    applicable = [
        rule
        for rule in RESTRICTION_RULES
        if rule.applies(item, subject_variable, context)
    ]
    return most_specific(applicable, key=lambda rule: rule.priority)


# ── restriction-subject rules (which variable does the WHERE restrict?) ──────


class RestrictionSubjectRule(ABC):
    """
    Resolve **which variable** a query's selection restricts, so the selection's ``WHERE``
    can fold into a post-nominal *"whose …"* modifier on it.  A selection matched by no
    rule has no groupable subject — its ``WHERE`` stays a full *"such that …"* clause.
    """

    priority: ClassVar[int] = 0
    """Tiebreak when several rules apply (higher wins)."""

    @classmethod
    @abstractmethod
    def applies(
        cls, expression, selected_variable, context: "VerbalizationContext"
    ) -> bool:
        """Return ``True`` when this rule can name the restriction subject of *expression*."""

    @classmethod
    @abstractmethod
    def subject(cls, expression, selected_variable, context: "VerbalizationContext"):
        """Return the :class:`Variable` the ``WHERE`` restricts."""


class SelectedVariableSubjectRule(RestrictionSubjectRule):
    """The selection is a plain :class:`Variable` → it is its own subject."""

    @classmethod
    def applies(
        cls, expression, selected_variable, context: "VerbalizationContext"
    ) -> bool:
        return isinstance(selected_variable, Variable)

    @classmethod
    def subject(cls, expression, selected_variable, context: "VerbalizationContext"):
        return selected_variable


class AggregationSourceSubjectRule(RestrictionSubjectRule):
    """
    The selection aggregates over a single source variable's chain (e.g.
    ``max(t.amount_details.amount)``); the ``WHERE`` restricts that aggregated entity,
    whose noun ends the selection so a *"whose …"* modifier attaches grammatically.
    """

    @classmethod
    def applies(
        cls, expression, selected_variable, context: "VerbalizationContext"
    ) -> bool:
        return (
            isinstance(selected_variable, Aggregator)
            and aggregation_source_root(expression) is not None
        )

    @classmethod
    def subject(cls, expression, selected_variable, context: "VerbalizationContext"):
        return aggregation_source_root(expression)


RESTRICTION_SUBJECT_RULES: List[Type[RestrictionSubjectRule]] = [
    SelectedVariableSubjectRule,
    AggregationSourceSubjectRule,
]


def restriction_subject(expression, selected_variable, context: "VerbalizationContext"):
    """The variable a selection's ``WHERE`` restricts (most-specific rule wins), or ``None``."""
    applicable = [
        rule
        for rule in RESTRICTION_SUBJECT_RULES
        if rule.applies(expression, selected_variable, context)
    ]
    chosen = most_specific(applicable, key=lambda rule: rule.priority)
    return (
        chosen.subject(expression, selected_variable, context)
        if chosen is not None
        else None
    )

from __future__ import annotations

import operator
from typing import TYPE_CHECKING

from krrood.entity_query_language.core.mapped_variable import Attribute, MappedVariable
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import (
    AND,
    OR,
    Not,
    LogicalOperator,
)
from krrood.entity_query_language.verbalization.fragments.base import (
    join_with,
    oxford_and,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.range_fold import (
    build_between,
    fold_range_pairs,
    has_pair,
    RangeFold,
)
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.subquery import is_calculation_value
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Conjunctions,
    Logicals,
    Operators,
)

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


def _word(text: str) -> VerbFragment:
    from krrood.entity_query_language.verbalization.fragments.base import WordFragment

    return WordFragment(text=text)


def _phrase(*parts: VerbFragment, sep: str = " ") -> PhraseFragment:
    return PhraseFragment(parts=list(parts), separator=sep)


def _is_bool_attr_chain(expr) -> bool:
    if not isinstance(expr, MappedVariable):
        return False
    from krrood.entity_query_language.verbalization.chain_utils import walk_chain

    chain, _ = walk_chain(expr)
    return bool(chain) and isinstance(chain[-1], Attribute) and chain[-1]._type_ is bool


class LogicalRule(VerbalizationRule):
    """
    Abstract base rule: catches any
    :class:`~krrood.entity_query_language.operators.core_logical_operators.LogicalOperator`.

    Concrete subclasses (:class:`AndRule`, :class:`OrRule`, :class:`NotRule`)
    handle specific operator types and take priority over this class due to MRO-depth
    sorting in :class:`~krrood.entity_query_language.verbalization.rule_engine.RuleEngine`.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for any :class:`~krrood.entity_query_language.operators.core_logical_operators.LogicalOperator`."""
        return isinstance(expr, LogicalOperator)


class AndRule(LogicalRule):
    """
    Verbalizes conjunctions (``AND(a, b, c)``) as *"a, b, and c"* using Oxford-comma style.

    Flattens nested AND chains before joining so that ``AND(AND(a,b),c)``
    produces *"a, b, and c"* rather than *"(a and b) and c"*.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for :class:`~krrood.entity_query_language.operators.core_logical_operators.AND` expressions."""
        return isinstance(expr, AND)

    @classmethod
    def transform(
        cls, expr: "AND", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Flatten the AND chain and join with Oxford-comma *"and"*.

        :param expr: Root AND expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Oxford-comma joined fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        parts = [delegate.build(c, ctx) for c in ctx.flatten_same_type(expr, AND)]
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())


class RangeConjunctionRule(AndRule):
    """
    Verbalizes a conjunction that contains a lower-bound / upper-bound pair on the
    same attribute by folding it into a *"… is between lo and hi"* phrase.

    Precondition (declarative): an ``AND`` whose flattened conjuncts contain at least
    one foldable pair (:func:`~krrood.entity_query_language.verbalization.range_fold.has_pair`).
    Takes priority over :class:`AndRule`; non-range conjunctions fall through.
    The left side of the range is verbalized normally, so it still picks up a
    pronoun (*"its booking_date is between …"*) when the chain root is the subject.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for an ``AND`` containing a foldable lo/hi range pair."""
        return isinstance(expr, AND) and has_pair(ctx.flatten_same_type(expr, AND))

    @classmethod
    def transform(
        cls, expr: "AND", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Fold range pairs and join the resulting items Oxford-comma style.

        :param expr: Root AND expression containing a range pair.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Conjunction fragment with folded *between* phrase(s).
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        parts: list[VerbFragment] = []
        for item in fold_range_pairs(ctx.flatten_same_type(expr, AND)):
            if isinstance(item, RangeFold):
                parts.append(
                    build_between(
                        delegate.build(item.chain_expr, ctx),
                        delegate.build(item.lo_expr, ctx),
                        delegate.build(item.hi_expr, ctx),
                        compact=ctx.compact_predicates,
                    )
                )
            else:
                parts.append(delegate.build(item, ctx))
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())


class OrRule(LogicalRule):
    """
    Verbalizes disjunctions as *"either a, b, or c"* using Oxford-comma style.

    Flattens nested OR chains before joining.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for :class:`~krrood.entity_query_language.operators.core_logical_operators.OR` expressions."""
        return isinstance(expr, OR)

    @classmethod
    def transform(
        cls, expr: "OR", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Flatten the OR chain and produce *"either a, b, or c"*.

        :param expr: Root OR expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Disjunction phrase fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        parts = [delegate.build(c, ctx) for c in ctx.flatten_same_type(expr, OR)]
        if len(parts) == 1:
            return parts[0]
        head_with_comma = PhraseFragment(
            parts=[join_with(parts[:-1], _word(", ")), _word(",")], separator=""
        )
        return _phrase(
            Logicals.EITHER.as_fragment(),
            head_with_comma,
            Conjunctions.OR.as_fragment(),
            parts[-1],
        )


class NotRule(LogicalRule):
    """
    Generic negation rule: wraps the child in *"not (<child>)"*.

    :class:`NotComparatorRule` and :class:`NotBoolAttrRule` take priority when
    they match (they are deeper in the MRO hierarchy).
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for :class:`~krrood.entity_query_language.operators.core_logical_operators.Not` expressions."""
        return isinstance(expr, Not)

    @classmethod
    def transform(
        cls, expr: "Not", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Build *"not (<child>)"*.

        :param expr: Not expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Negation phrase fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        child_frag = delegate.build(expr._child_, ctx)
        return _phrase(
            Logicals.NOT.as_fragment(),
            PhraseFragment(parts=[_word("("), child_frag, _word(")")], separator=""),
        )


class NotComparatorRule(NotRule):
    """
    Negates a Comparator inline: *"a is not greater than b"* instead of *"not (a is greater than b)"*.

    Applies when the Not child is a
    :class:`~krrood.entity_query_language.operators.comparator.Comparator`.
    Uses :meth:`~krrood.entity_query_language.verbalization.vocabulary.english.Operators.from_callable`
    with ``negated=True`` to select the negated operator phrase.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` when the Not child is a Comparator."""
        return isinstance(expr, Not) and isinstance(expr._child_, Comparator)

    @classmethod
    def transform(
        cls, expr: "Not", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Build *"<left> <negated_op> <right>"*.

        :param expr: Not-wrapping-Comparator expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Negated comparator phrase.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        child = expr._child_
        left = delegate.build(child.left, ctx)
        right = delegate.build(child.right, ctx)
        is_temporal = delegate._chain.is_temporal(
            child.left
        ) or delegate._chain.is_temporal(child.right)
        try:
            op_frag = (
                Operators.from_callable(child.operation)
                .select(
                    negated=True, compact=ctx.compact_predicates, temporal=is_temporal
                )
                .as_fragment()
            )
        except KeyError:
            op_frag = RoleFragment.for_operator(f"not {child._name_}")
        return _phrase(left, op_frag, right)


class NotCalculationEqualityRule(NotComparatorRule):
    """
    Negates a calc-equality inline: *"<left> is not equal to <right>"* (for
    ``not (a == calc)``) or *"<left> is equal to <right>"* (for ``not (a != calc)``).

    Precondition (declarative): the Not child is a ``==``/``!=`` Comparator with a
    calculation operand
    (:func:`~krrood.entity_query_language.verbalization.subquery.is_calculation_value`).
    Takes priority over :class:`NotComparatorRule` via MRO depth.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` when the Not child is a calc-equality comparator."""
        if not (isinstance(expr, Not) and isinstance(expr._child_, Comparator)):
            return False
        child = expr._child_
        return child.operation in (operator.eq, operator.ne) and (
            is_calculation_value(child.left) or is_calculation_value(child.right)
        )

    @classmethod
    def transform(
        cls, expr: "Not", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Build the negated calc-equality phrase.

        :param expr: Not-wrapping calc-equality Comparator.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Negated calc-equality fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        child = expr._child_
        left = delegate.build(child.left, ctx)
        right = delegate.build(child.right, ctx)
        op_frag = Operators.CALC_EQ.select(
            negated=child.operation is operator.eq, compact=ctx.compact_predicates
        ).as_fragment()
        return _phrase(left, op_frag, right)


class NotBoolAttrRule(NotRule):
    """
    Negates a boolean attribute chain: *"<nav> is not <attr>"*.

    Applies when the Not child is a
    :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
    chain whose terminal node is a ``bool``-typed
    :class:`~krrood.entity_query_language.core.mapped_variable.Attribute`.
    Delegates to :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_mapped_negated`.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` when the Not child is a bool-typed Attribute chain."""
        return isinstance(expr, Not) and _is_bool_attr_chain(expr._child_)

    @classmethod
    def transform(
        cls, expr: "Not", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Delegate to :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_mapped_negated`.

        :param expr: Not-wrapping-bool-Attribute expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Predicative *"is not <attr>"* fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        return delegate._chain.verbalize_mapped_negated(expr._child_, ctx)

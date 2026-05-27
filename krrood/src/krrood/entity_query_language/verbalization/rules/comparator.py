from __future__ import annotations

import operator
from typing import TYPE_CHECKING

from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
from krrood.entity_query_language.verbalization.operator_phrase import comparator_phrase
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule
from krrood.entity_query_language.verbalization.subquery import is_calculation_value

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


class ComparatorRule(VerbalizationRule):
    """
    Verbalizes :class:`~krrood.entity_query_language.operators.comparator.Comparator`
    expressions as *"<left> <operator> <right>"*.

    Selects the correct operator phrase via
    :meth:`~krrood.entity_query_language.verbalization.vocabulary.english.Operators.from_callable`
    using the ``compact`` and ``temporal`` flags from the context.

    Falls back to ``expr._name_`` as a plain operator fragment when the
    callable is not registered in
    :class:`~krrood.entity_query_language.verbalization.vocabulary.english.Operators`.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for :class:`~krrood.entity_query_language.operators.comparator.Comparator` expressions."""
        return isinstance(expr, Comparator)

    @classmethod
    def transform(
        cls, expr: "Comparator", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Build *"<left> <operator> <right>"*.

        :param expr: Comparator expression.
        :param ctx: Shared verbalization state (``compact_predicates`` used for HAVING clauses).
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Comparison phrase fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        return comparator_phrase(expr, ctx, delegate)


class CalculationEqualityRule(ComparatorRule):
    """
    Verbalizes an equality/inequality whose other operand is a *calculation*
    (an aggregation) as *"<left> is equal to <right>"* / *"<left> is not equal to
    <right>"* rather than the plain *"is"* / *"is not"*.

    Precondition (declarative): the operation is ``==``/``!=`` and either operand
    :func:`~krrood.entity_query_language.verbalization.subquery.is_calculation_value`.
    Takes priority over :class:`ComparatorRule` via MRO depth; object/value equalities
    keep the bare *"is"* by falling through.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for ``==``/``!=`` against a calculation operand."""
        return (
            isinstance(expr, Comparator)
            and expr.operation in (operator.eq, operator.ne)
            and (is_calculation_value(expr.left) or is_calculation_value(expr.right))
        )

    @classmethod
    def transform(
        cls, expr: "Comparator", ctx: "VerbalizationContext", delegate: "EQLVerbalizer"
    ) -> VerbFragment:
        """
        Build *"<left> is [not] equal to <right>"* using the calc-equality phrase.

        :param expr: Comparator (``==``/``!=``) with a calculation operand.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer for recursive calls.
        :returns: Calc-equality comparison fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        return comparator_phrase(expr, ctx, delegate)

from __future__ import annotations

from typing import TYPE_CHECKING

from krrood.entity_query_language.core.mapped_variable import (
    Attribute,
    FlatVariable,
    MappedVariable,
)
from krrood.entity_query_language.verbalization.chain_utils import walk_chain
from krrood.entity_query_language.verbalization.fragments.base import VerbFragment
from krrood.entity_query_language.verbalization.rule_engine import VerbalizationRule

if TYPE_CHECKING:
    from krrood.entity_query_language.verbalization.context import VerbalizationContext
    from krrood.entity_query_language.verbalization.verbalizer import EQLVerbalizer


class MappedVariableRule(VerbalizationRule):
    """
    Verbalizes all :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
    chains (Attribute, Index, Call) by delegating to
    :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_mapped`.

    :class:`FlatVariableRule` handles the special
    :class:`~krrood.entity_query_language.core.mapped_variable.FlatVariable` subclass
    and takes priority.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for any :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`."""
        return isinstance(expr, MappedVariable)

    @classmethod
    def transform(
        cls,
        expr: "MappedVariable",
        ctx: "VerbalizationContext",
        delegate: "EQLVerbalizer",
    ) -> VerbFragment:
        """
        Delegate to :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_mapped`.

        :param expr: Root of a MappedVariable chain.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer.
        :returns: Possessive or predicative chain fragment.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        return delegate._chain.verbalize_mapped(expr, ctx)


class PronominalChainRule(MappedVariableRule):
    """
    Verbalizes a :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable`
    chain whose root is the current coreference subject using a possessive pronoun:
    *"the amount of its amount_details"* / *"its booking_date"* instead of
    *"… of the BankTransaction"*.

    Precondition (declarative): the chain has a non-boolean terminal attribute and its
    root is the unambiguous, already-introduced subject — i.e.
    :meth:`~krrood.entity_query_language.verbalization.context.VerbalizationContext.pronoun_for`
    returns a fragment.  Takes priority over :class:`MappedVariableRule` via MRO depth;
    when the precondition fails it falls through to the plain possessive-path form.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for a non-bool chain rooted at the current pronoun-eligible subject."""
        if not isinstance(expr, MappedVariable) or isinstance(expr, FlatVariable):
            return False
        chain, root = walk_chain(expr)
        if not chain:
            return False
        terminal = chain[-1]
        if isinstance(terminal, Attribute) and terminal._type_ is bool:
            return False
        return ctx.pronoun_for(root) is not None

    @classmethod
    def transform(
        cls,
        expr: "MappedVariable",
        ctx: "VerbalizationContext",
        delegate: "EQLVerbalizer",
    ) -> VerbFragment:
        """
        Delegate to :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_possessive`.

        :param expr: Root of a MappedVariable chain rooted at the current subject.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer.
        :returns: Possessive chain fragment using the *"its"* pronoun.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        _chain, root = walk_chain(expr)
        return delegate._chain.verbalize_possessive(expr, ctx, ctx.pronoun_for(root))


class FlatVariableRule(MappedVariableRule):
    """
    Verbalizes :class:`~krrood.entity_query_language.core.mapped_variable.FlatVariable`
    by unwrapping to its child expression.

    :class:`~krrood.entity_query_language.core.mapped_variable.FlatVariable` is a transparent
    wrapper added during SetOf expansion.  It is a subclass of
    :class:`~krrood.entity_query_language.core.mapped_variable.MappedVariable` and takes
    priority over :class:`MappedVariableRule` via MRO-depth sorting.
    """

    @classmethod
    def applies(cls, expr, ctx: "VerbalizationContext") -> bool:
        """Return ``True`` for :class:`~krrood.entity_query_language.core.mapped_variable.FlatVariable`."""
        return isinstance(expr, FlatVariable)

    @classmethod
    def transform(
        cls,
        expr: "FlatVariable",
        ctx: "VerbalizationContext",
        delegate: "EQLVerbalizer",
    ) -> VerbFragment:
        """
        Delegate to :meth:`~krrood.entity_query_language.verbalization.chain_verbalizer.ChainVerbalizer.verbalize_flat`.

        :param expr: FlatVariable expression.
        :param ctx: Shared verbalization state.
        :param delegate: Parent verbalizer.
        :returns: Fragment for the child expression.
        :rtype: ~krrood.entity_query_language.verbalization.fragments.base.VerbFragment
        """
        return delegate._chain.verbalize_flat(expr, ctx)

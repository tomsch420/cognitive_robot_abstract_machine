"""
The English grammar — one :class:`~krrood.entity_query_language.verbalization.grammar.phrase_rule.PhraseRule`
subclass per EQL construct.

Each rule is a Montague rule-to-rule clause: *for this construct, build this phrase*
(Montague 1970; Bach 1976, the rule-to-rule hypothesis).  A rule only **combines** —
recursion is delegated to ``ctx.child`` (the fold), and cross-cutting decisions to the
microplanning services (``ctx.refer`` / ``ctx.scope`` / ``ctx.config``), morphology, the
coordination module, and the lexicon — so each rule is responsible for a single
construct's surface composition.

:data:`RULES` lists one instance per rule; the registry exposes them as queryable
data.  Families are ported here one at a time; until a construct's rule is present the
engine falls back to the legacy dispatcher (strangler migration).
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import List

from krrood.entity_query_language.core.mapped_variable import Attribute, MappedVariable
from krrood.entity_query_language.core.variable import (
    ExternallySetVariable,
    Literal,
    Variable,
)
from krrood.entity_query_language.operators.comparator import Comparator
from krrood.entity_query_language.operators.core_logical_operators import (
    AND,
    OR,
    Not,
    flatten_operands,
)
from krrood.entity_query_language.verbalization.chain_utils import walk_chain
from krrood.entity_query_language.verbalization.fragments.base import (
    join_with,
    oxford_and,
    PhraseFragment,
    RoleFragment,
    VerbFragment,
)
from krrood.entity_query_language.verbalization.fragments.factory import (
    phrase,
    role,
    word,
)
from krrood.entity_query_language.verbalization.fragments.roles import SemanticRole
from krrood.entity_query_language.verbalization.grammar.phrase_rule import (
    Ctx,
    PhraseRule,
)
from krrood.entity_query_language.verbalization.microplanning.coordination import (
    build_between,
    fold_range_pairs,
    has_pair,
    RangeFold,
)
from krrood.entity_query_language.verbalization.microplanning.referring import (
    ArticleSelection,
)
from krrood.entity_query_language.verbalization.operator_phrase import (
    comparator_operator,
    comparator_phrase,
)
from krrood.entity_query_language.verbalization.rules.chains import verbalize_chain
from krrood.entity_query_language.verbalization.vocabulary.english import (
    Articles,
    Conjunctions,
    Logicals,
)


@dataclass
class _FoldVerbalizer:
    """
    Adapter exposing ``build(expression, context)`` over a :class:`Ctx`, for the few
    legacy helpers (``comparator_phrase``, ``verbalize_chain``) that still expect a
    verbalizer object.  Children recurse through ``ctx.child`` (the fold).  Transitional —
    removed once those helpers are absorbed into the grammar.
    """

    ctx: Ctx

    def build(self, expression, context=None) -> VerbFragment:
        return self.ctx.child(expression)


def _is_bool_attr_chain(expression) -> bool:
    """``True`` when *expression* is a MappedVariable chain ending in a bool-typed Attribute."""
    if not isinstance(expression, MappedVariable):
        return False
    chain, _ = walk_chain(expression)
    return bool(chain) and isinstance(chain[-1], Attribute) and chain[-1]._type_ is bool


# ── comparator ───────────────────────────────────────────────────────────────


class ComparatorRule(PhraseRule):
    """``<left> <operator> <right>`` — e.g. *"is greater than 50"*."""

    construct = Comparator
    name = "comparator"

    def build(self, node, ctx: Ctx):
        return phrase(
            ctx.child(node.left),
            comparator_operator(node, ctx.context),
            ctx.child(node.right),
        )


# ── variables ──────────────────────────────────────────────────────────────────


class VariableRule(PhraseRule):
    """*"a/an Robot"* (first mention), *"the Robot"* (subsequent), or *"Robot N"* (numbered)."""

    construct = Variable
    name = "variable"

    def build(self, node, ctx: Ctx):
        article, label = ctx.refer.noun_for_parts(node)
        label_fragment = RoleFragment.for_variable(label, node)
        if article == ArticleSelection.NONE:
            return label_fragment
        if article == ArticleSelection.DEFINITE:
            return phrase(Articles.THE.as_fragment(), label_fragment)
        return phrase(Articles.indefinite(label), label_fragment)


class LiteralRule(PhraseRule):
    """A literal value (``Literal <: Variable`` — deeper construct wins via ``select``)."""

    construct = Literal
    name = "literal"

    def build(self, node, ctx: Ctx):
        return role(ctx.context.type_name_of_value(node._value_), SemanticRole.LITERAL)


class ExternalVariableRule(PhraseRule):
    """*"a/an TypeName"* for an opaque externally-set variable (no coreference)."""

    construct = ExternallySetVariable
    name = "external-variable"

    def build(self, node, ctx: Ctx):
        type_name = (
            node._type_.__name__ if getattr(node, "_type_", None) else "variable"
        )
        return phrase(
            Articles.indefinite(type_name), role(type_name, SemanticRole.VARIABLE)
        )


# ── logical ──────────────────────────────────────────────────────────────────


class AndRule(PhraseRule):
    """Conjunction *"a, b, and c"* (Oxford comma); flattens nested ANDs."""

    construct = AND
    name = "and"

    def build(self, node, ctx: Ctx):
        parts = [ctx.child(conjunct) for conjunct in flatten_operands(node, AND)]
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())


class RangeConjunctionRule(PhraseRule):
    """A conjunction containing a lo/hi pair on one chain → *"… is between lo and hi"*."""

    construct = AND
    name = "and-range"

    def when(self, node):
        return has_pair(flatten_operands(node, AND))

    def build(self, node, ctx: Ctx):
        parts: List[VerbFragment] = []
        for item in fold_range_pairs(flatten_operands(node, AND)):
            if isinstance(item, RangeFold):
                parts.append(
                    build_between(
                        ctx.child(item.chain_expression),
                        ctx.child(item.lower_expression),
                        ctx.child(item.upper_expression),
                        compact=ctx.config.compact_predicates,
                    )
                )
            else:
                parts.append(ctx.child(item))
        if len(parts) == 1:
            return parts[0]
        return oxford_and(parts, Conjunctions.AND.as_fragment())


class OrRule(PhraseRule):
    """Disjunction *"either a, b, or c"*; flattens nested ORs."""

    construct = OR
    name = "or"

    def build(self, node, ctx: Ctx):
        parts = [ctx.child(conjunct) for conjunct in flatten_operands(node, OR)]
        if len(parts) == 1:
            return parts[0]
        head_with_comma = PhraseFragment(
            parts=[join_with(parts[:-1], word(", ")), word(",")], separator=""
        )
        return phrase(
            Logicals.EITHER.as_fragment(),
            head_with_comma,
            Conjunctions.OR.as_fragment(),
            parts[-1],
        )


class NotRule(PhraseRule):
    """Generic negation *"not (<child>)"* (specialised by the guarded Not rules below)."""

    construct = Not
    name = "not"

    def build(self, node, ctx: Ctx):
        child_fragment = ctx.child(node._child_)
        return phrase(
            Logicals.NOT.as_fragment(),
            PhraseFragment(parts=[word("("), child_fragment, word(")")], separator=""),
        )


class NotComparatorRule(PhraseRule):
    """Inline negated comparator *"a is not greater than b"* (Not over a Comparator)."""

    construct = Not
    name = "not-comparator"

    def when(self, node):
        return isinstance(node._child_, Comparator)

    def build(self, node, ctx: Ctx):
        return comparator_phrase(
            node._child_, ctx.context, _FoldVerbalizer(ctx), negated=True
        )


class NotBoolAttrRule(PhraseRule):
    """Negated boolean attribute chain *"<nav> is not <attr>"* (Not over a bool-attr chain)."""

    construct = Not
    name = "not-bool-attr"

    def when(self, node):
        return _is_bool_attr_chain(node._child_)

    def build(self, node, ctx: Ctx):
        return verbalize_chain(
            node._child_, ctx.context, _FoldVerbalizer(ctx), negated=True
        )


# ── the full grammar (one instance per rule) ─────────────────────────────────────

RULES: List[PhraseRule] = [
    ComparatorRule(),
    VariableRule(),
    LiteralRule(),
    ExternalVariableRule(),
    AndRule(),
    RangeConjunctionRule(),
    OrRule(),
    NotRule(),
    NotComparatorRule(),
    NotBoolAttrRule(),
]

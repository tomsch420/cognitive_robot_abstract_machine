from __future__ import annotations

import uuid
from dataclasses import dataclass

from typing_extensions import Any, Dict, FrozenSet, List, Mapping

from krrood.entity_query_language.core.base_expressions import SymbolicExpression
from krrood.entity_query_language.core.variable import Literal, Variable
from krrood.entity_query_language.verbalization import morphology
from krrood.entity_query_language.verbalization.fragments.base import (
    NounPhrase,
    RoleFragment,
    VerbalizationFragment,
    WordFragment,
)
from krrood.entity_query_language.verbalization.fragments.features import Definiteness
from krrood.entity_query_language.verbalization.value_lexicon import type_noun
from krrood.patterns.field_metadata import GrammarMetadata

GENERIC_OPERAND_NAMES: FrozenSet[str] = frozenset(
    {
        "node",
        "obj",
        "object",
        "entity",
        "thing",
        "value",
        "self",
        "variable",
        "item",
        "element",
        "arg",
        "argument",
        "operand",
        "instance",
        "x",
    }
)
"""
Operand field names that name a *slot* rather than a domain role: they carry no more
information than the operand's type, so naming falls through them to the type name.

This keeps the modelling burden low — a predicate whose field is a generic placeholder
reads as its type without any metadata — while a field with a meaningful role name
(*"body"*, *"camera"*) names the operand directly.
"""

_ORDINAL_FIELD_PREFIXES: FrozenSet[str] = frozenset(
    {
        "first",
        "second",
        "third",
        "fourth",
        "fifth",
        "sixth",
        "seventh",
        "eighth",
        "ninth",
        "tenth",
    }
)
"""
Leading ordinal words that only disambiguate two same-type operands (*"first_entity"* /
*"second_entity"*); stripped so both operands collapse to one shared noun and are told
apart by a determiner (*"an entity … the other entity"*) instead.
"""


def clean_field_name(field_name: str) -> str:
    """:return: *field_name* as a surface noun with the disambiguation markers of a same-type operand
    pair removed — a trailing numeric suffix (*"point_1"* → *"point"*) and a leading ordinal word
    (*"first_entity"* → *"entity"*) — and the remaining underscores read as spaces.

    Both strippings make two same-type operands resolve to the *same* noun, which is what lets the
    determiner disambiguation (*"a point … the other point"*) replace numbering.

    >>> clean_field_name("point_1")
    'point'
    >>> clean_field_name("first_entity")
    'entity'
    >>> clean_field_name("amount_details")
    'amount details'
    """
    parts = [part for part in field_name.split("_") if part]
    if parts and parts[-1].isdigit():
        parts = parts[:-1]
    if len(parts) > 1 and parts[0].lower() in _ORDINAL_FIELD_PREFIXES:
        parts = parts[1:]
    return " ".join(parts)


def operand_noun(callable_class: type, field_name: str, operand_type: Any) -> str:
    """:return: the noun naming one operand, resolved in order of decreasing specificity:
    the field's grammatical-metadata display name, then the field name itself (unless it is a
    generic slot placeholder), then the operand's type noun as a last resort.

    :param callable_class: The predicate / function class owning the field (for its metadata).
    :param field_name: The operand's dataclass field name.
    :param operand_type: The operand variable's type, used for the last-resort type noun.

    >>> operand_noun(type("FacesTowards", (), {}), "point_1", object)
    'point'
    """
    metadata = GrammarMetadata.of_field(callable_class, field_name)
    if metadata is not None and metadata.display_name is not None:
        return metadata.display_name
    cleaned = clean_field_name(field_name)
    if cleaned and cleaned.lower() not in GENERIC_OPERAND_NAMES:
        return cleaned
    return type_noun(operand_type) if isinstance(operand_type, type) else "object"


def is_anonymous_operand(
    child: SymbolicExpression, occurrences: Mapping[uuid.UUID, int]
) -> bool:
    """:return: whether *child* is an anonymous operand — a plain variable (not a literal) referenced
    exactly once in the whole expression, so it has no independent discourse identity and is named by
    the field it fills rather than by its type. A variable reused elsewhere (a query subject, a
    constrained variable) is referenced more than once and keeps its normal referring rendering.

    :param child: The expression bound to the operand field.
    :param occurrences: Reference count per node id across the whole expression.
    """
    if not isinstance(child, Variable) or isinstance(child, Literal):
        return False
    return occurrences.get(child._id_) == 1


@dataclass(frozen=True)
class OperandNaming:
    """
    Names a symbolic callable's operands from its fields, so a predicate reads *"a body
    is reachable"* (the field) rather than *"an object is reachable"* (the type), and
    two same-type operands are told apart by a determiner (*"a point … the other
    point"*) rather than a number.

    Only anonymous operands are renamed; an operand that also appears elsewhere (a query
    subject) keeps its type-named referring expression so coreference still
    pronominalises it.
    """

    callable_class: type
    """
    The predicate / function class whose operands are being named.
    """

    occurrences: Mapping[uuid.UUID, int]
    """
    Reference count per node id, used to tell an anonymous operand from a reused
    variable.
    """

    def operand_fragments(
        self, children: Mapping[str, SymbolicExpression]
    ) -> Dict[str, VerbalizationFragment]:
        """:return: a field-named noun phrase for each anonymous operand, keyed by field name; a field
        whose operand is not anonymous is absent, so the caller renders it normally.

        Anonymous operands are grouped by their resolved noun: a group of one reads *"a point"*, a
        group of several is disambiguated by determiners — *"a point"*, *"the other point"*, *"the
        third point"*.

        :param children: The callable's field name → bound child expression mapping.
        """
        anonymous = {
            name: child
            for name, child in children.items()
            if is_anonymous_operand(child, self.occurrences)
        }
        nouns = {
            name: operand_noun(self.callable_class, name, child._type_)
            for name, child in anonymous.items()
        }
        groups: Dict[str, List[str]] = {}
        for name in anonymous:
            groups.setdefault(nouns[name], []).append(name)
        fragments: Dict[str, VerbalizationFragment] = {}
        for group in groups.values():
            for index, name in enumerate(group):
                fragments[name] = _operand_noun_phrase(
                    anonymous[name]._type_, nouns[name], index, len(group)
                )
        return fragments


def _operand_noun_phrase(
    operand_type: Any, noun: str, index: int, group_size: int
) -> NounPhrase:
    """:return: the noun phrase for one operand of a same-noun group — the first (or a lone operand)
    is indefinite (*"a point"*), a second is *"the other point"*, and any further operand takes an
    ordinal (*"the third point"*).

    The head keeps the operand's type source link (so *"point"* still hyperlinks to its class) while
    displaying the resolved noun.
    """
    head = RoleFragment.for_type(operand_type, text=noun)
    if group_size == 1 or index == 0:
        return NounPhrase(head=head, definiteness=Definiteness.INDEFINITE)
    qualifier = "other" if group_size == 2 else morphology.ordinal(index)
    return NounPhrase(
        head=head,
        definiteness=Definiteness.DEFINITE,
        pre_head=WordFragment(text=qualifier),
    )

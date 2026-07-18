"""
Tests for operand naming: how a predicate / function names the operands filling its
fields.

An anonymous operand — a variable that appears only as this operand — is named by its
field (grammatical metadata, then the field name, then the type as a last resort) rather
than always by its type, and two same-named operands are told apart by a determiner (*"a
point … the other point"*) rather than a number. A variable reused elsewhere (a query
subject) keeps its type-named referring expression so coreference still pronominalises
it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from krrood.entity_query_language.factories import an, entity, for_all, variable
from krrood.entity_query_language.predicate import Predicate, SymbolicFunction
from krrood.entity_query_language.verbalization.microplanning.operand_naming import (
    GENERIC_OPERAND_NAMES,
    OperandNaming,
    clean_field_name,
    is_anonymous_operand,
    operand_noun,
)
from krrood.entity_query_language.verbalization.microplanning.referring import (
    ReferringExpressions,
)
from krrood.entity_query_language.verbalization.pipeline import verbalize_expression
from krrood.entity_query_language.verbalization.vocabulary.english import Prepositions
from krrood.entity_query_language.verbalization.vocabulary.parts_of_speech import (
    Adjective,
    clause,
    Copula,
    ConjunctivePhrase,
    FunctionVerbalizationTemplates,
    Noun,
    Verb,
)
from krrood.patterns.field_metadata import FieldMetadata, GrammarMetadata


class Marker:
    """
    A stand-in operand type whose class name is unremarkable, so the field name drives
    the surface noun.
    """


class Worker:
    """
    A stand-in operand type for the distinct-role predicate.
    """


class Device:
    """
    A stand-in operand type behind a generic slot field name.
    """


@dataclass(eq=False)
class SameNamePair(Predicate):
    """
    Two same-type operands whose field names differ only by a numeric suffix, so they
    resolve to one noun disambiguated by a determiner.
    """

    point_1: Marker
    point_2: Marker

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["point_1"]), Verb("face"), Noun(fields["point_2"]))


@dataclass(eq=False)
class SameNameTriple(Predicate):
    """
    Three same-named operands, disambiguated by determiner then ordinal.
    """

    point_1: Marker
    point_2: Marker
    point_3: Marker

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(
            ConjunctivePhrase(
                [
                    Noun(fields["point_1"]),
                    Noun(fields["point_2"]),
                    Noun(fields["point_3"]),
                ]
            ),
            Copula(),
            Adjective("collinear"),
        )


@dataclass(eq=False)
class MetadataNamed(Predicate):
    """
    An operand named by a grammatical-metadata display name, overriding both its field
    name and its type.
    """

    burning_thing: object = field(
        metadata=FieldMetadata(
            other_metadata=[GrammarMetadata(display_name="torch")]
        ).as_dict()
    )

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["burning_thing"]), Copula(), Adjective("lit"))


@dataclass(eq=False)
class GenericSlot(Predicate):
    """
    An operand whose field name is a generic slot placeholder, so the surface falls
    through to the type name.
    """

    node: Device

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["node"]), Copula(), Adjective("idle"))


@dataclass(eq=False)
class DistinctRoles(Predicate):
    """
    Two same-type operands with distinct role names, kept distinct (no determiner
    disambiguation).
    """

    subordinate: Worker
    supervisor: Worker

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(
            Noun(fields["subordinate"]),
            Verb("report"),
            Prepositions.TO,
            Noun(fields["supervisor"]),
        )


@dataclass(eq=False)
class SingleRole(Predicate):
    """
    A single-operand predicate used both standalone (field-named) and as a query subject
    (type-named, pronominalised).
    """

    surface: Marker

    def __call__(self) -> bool:
        return True

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return clause(Noun(fields["surface"]), Copula(), Adjective("warm"))


@dataclass(eq=False)
class RoleNamedReading(SymbolicFunction):
    """
    A value function whose operand is field-named inside a possessive noun phrase.
    """

    sensor: object

    def __call__(self) -> float:
        return 0.0

    @classmethod
    def _verbalization_fragment_(cls, fields):
        return FunctionVerbalizationTemplates.possessive(cls, *fields.values())


# ── field-name resolution ────────────────────────────────────────────────────────


def test_clean_field_name_strips_numeric_suffix():
    assert clean_field_name("point_1") == "point"


def test_clean_field_name_strips_leading_ordinal():
    assert clean_field_name("first_entity") == "entity"


def test_clean_field_name_reads_underscores_as_spaces():
    assert clean_field_name("amount_details") == "amount details"


def test_operand_noun_prefers_metadata_display_name():
    assert operand_noun(MetadataNamed, "burning_thing", object) == "torch"


def test_operand_noun_uses_field_name_when_meaningful():
    assert operand_noun(DistinctRoles, "subordinate", Worker) == "subordinate"


def test_operand_noun_falls_through_generic_slot_to_type():
    assert operand_noun(GenericSlot, "node", Device) == "Device"
    assert "node" in GENERIC_OPERAND_NAMES


# ── anonymity ────────────────────────────────────────────────────────────────────


def test_single_use_variable_is_anonymous():
    predicate = SingleRole(variable(Marker, []))
    occurrences = ReferringExpressions.from_expression(predicate).occurrences
    surface = predicate._child_vars_["surface"]
    assert is_anonymous_operand(surface, occurrences)


def test_reused_variable_is_not_anonymous():
    subject = variable(Marker, [])
    query = an(entity(subject).where(SingleRole(subject)))
    occurrences = ReferringExpressions.from_expression(query).occurrences
    assert not is_anonymous_operand(subject, occurrences)


def test_occurrence_count_counts_each_reference():
    subject = variable(Marker, [])
    query = an(entity(subject).where(SingleRole(subject)))
    occurrences = ReferringExpressions.from_expression(query).occurrences
    assert occurrences[subject._id_] == 2


# ── end-to-end surfaces ──────────────────────────────────────────────────────────


def test_same_type_pair_uses_determiner_not_numbering():
    assert verbalize_expression(
        SameNamePair(variable(Marker, []), variable(Marker, []))
    ) == ("a point faces the other point")


def test_same_type_triple_uses_determiner_then_ordinals():
    assert verbalize_expression(
        SameNameTriple(variable(Marker, []), variable(Marker, []), variable(Marker, []))
    ) == ("a point, the second point, and the third point are collinear")


def test_metadata_display_name_names_the_operand():
    assert verbalize_expression(MetadataNamed(variable(object, []))) == "a torch is lit"


def test_generic_slot_field_falls_through_to_type():
    assert verbalize_expression(GenericSlot(variable(Device, []))) == "a Device is idle"


def test_distinct_role_names_are_kept_distinct():
    assert verbalize_expression(
        DistinctRoles(variable(Worker, []), variable(Worker, []))
    ) == ("a subordinate reports to a supervisor")


def test_anonymous_operand_is_named_by_its_field():
    assert verbalize_expression(SingleRole(variable(Marker, []))) == "a surface is warm"


def test_reused_operand_keeps_type_and_pronominalises():
    """
    A variable that is also the query subject keeps its type name and coreference — it
    is *not* replaced by the field name.
    """
    subject = variable(Marker, [])
    assert verbalize_expression(an(entity(subject).where(SingleRole(subject)))) == (
        "Find a Marker such that it is warm"
    )


def test_function_operand_is_field_named():
    assert verbalize_expression(RoleNamedReading(variable(object, []))) == (
        "the role named reading of a sensor"
    )


def test_operand_naming_leaves_reused_operand_to_normal_rendering():
    """
    The naming component returns an override only for anonymous operands; a reused
    operand is absent, so the rule renders it normally.
    """
    subject = variable(Marker, [])
    query = an(entity(subject).where(SingleRole(subject)))
    occurrences = ReferringExpressions.from_expression(query).occurrences
    naming = OperandNaming(SingleRole, occurrences)
    assert naming.operand_fragments({"surface": subject}) == {}

"""
Registry of all concrete :class:`~krrood.entity_query_language.verbalization.rule_engine.VerbalizationRule`
subclasses used by the :class:`~krrood.entity_query_language.verbalization.rule_engine.RuleEngine`.

:data:`ALL_RULES` is the single authoritative list consumed by
:class:`~krrood.entity_query_language.verbalization.verbalizer.EQLVerbalizer`.
Rules are ordered so that more-specific patterns appear before their parent classes;
the :class:`~krrood.entity_query_language.verbalization.rule_engine.RuleEngine` re-sorts
them by MRO depth anyway, but the explicit ordering here aids readability.
"""

from __future__ import annotations

from krrood.entity_query_language.verbalization.rules.logical import (
    LogicalRule,
    AndRule,
    RangeConjunctionRule,
    OrRule,
    NotRule,
    NotComparatorRule,
    NotCalculationEqualityRule,
    NotBoolAttrRule,
)
from krrood.entity_query_language.verbalization.rules.quantifiers import (
    QuantifierRule,
    ForAllRule,
    ExistsRule,
)
from krrood.entity_query_language.verbalization.rules.comparator import (
    ComparatorRule,
    CalculationEqualityRule,
)
from krrood.entity_query_language.verbalization.rules.aggregators import (
    AggregatorRule,
    CountAllRule,
)
from krrood.entity_query_language.verbalization.rules.variables import (
    VariableRule,
    LiteralRule,
    ExternallySetVariableRule,
    InstantiatedVariableRule,
    InstantiatedVerbalizableRule,
)
from krrood.entity_query_language.verbalization.rules.chains import (
    MappedVariableRule,
    PronominalChainRule,
    FlatVariableRule,
)
from krrood.entity_query_language.verbalization.rules.query import (
    EntityRule,
    SetOfRule,
    ResultQuantifierRule,
    FilterRule,
    GroupedByRule,
    OrderedByRule,
)
from krrood.entity_query_language.verbalization.rules.inference_rule import (
    InferenceRuleRule,
)

ALL_RULES: list[type] = [
    # logical — specific patterns first, generic Not last
    NotCalculationEqualityRule,
    NotComparatorRule,
    NotBoolAttrRule,
    NotRule,
    RangeConjunctionRule,
    AndRule,
    OrRule,
    # quantifiers
    ForAllRule,
    ExistsRule,
    # comparator — calc-equality before the generic comparator
    CalculationEqualityRule,
    ComparatorRule,
    # instantiated variables — template form before natural form
    InstantiatedVerbalizableRule,
    InstantiatedVariableRule,
    # plain variables — Literal before Variable
    LiteralRule,
    ExternallySetVariableRule,
    VariableRule,
    # aggregators — CountAll before generic Aggregator
    CountAllRule,
    AggregatorRule,
    # query structures — inference-rule form before the generic Entity form
    InferenceRuleRule,
    EntityRule,
    SetOfRule,
    ResultQuantifierRule,
    FilterRule,
    GroupedByRule,
    OrderedByRule,
    # mapped chains — Pronominal / FlatVariable before MappedVariable (via subclass depth)
    PronominalChainRule,
    FlatVariableRule,
    MappedVariableRule,
]

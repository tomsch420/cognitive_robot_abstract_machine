"""
Query **planner** — pure structural analysis of an
:class:`~krrood.entity_query_language.query.query.Entity` / :class:`~krrood.entity_query_language.query.query.SetOf`
into a :class:`QueryPlan` (the *what to say* decisions).

It never builds fragments, mutates the context, or recurses — those are realisation
concerns owned by
:class:`~krrood.entity_query_language.verbalization.grammar.assembly.query.QueryAssembler`.
The planner records: the selection shape, the definiteness (``is_the``), the restriction
subject and its WHERE partition (grouped *"whose …"* predicates vs. residual *"such
that …"*), the grouping keys + aggregated expressions, the HAVING/ORDER clauses, and
whether the entity is an aggregation value-subquery.

Reference: Reiter & Dale (2000) — content/structure determination (microplanning).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto

from typing_extensions import Any, List, Optional, Tuple, Type

from krrood.entity_query_language.core.variable import InstantiatedVariable, Variable
from krrood.entity_query_language.operators.core_logical_operators import (
    AND,
    flatten_operands,
)
from krrood.entity_query_language.query.quantifiers import The
from krrood.entity_query_language.query.query import Entity, Query, SetOf
from krrood.entity_query_language.verbalization.chain_utils import chain_root
from krrood.entity_query_language.verbalization.grammar.planning.base import Planner
from krrood.entity_query_language.verbalization.grammar.restriction import (
    RestrictionRule,
    match_restriction,
    restriction_subject,
)
from krrood.entity_query_language.verbalization.microplanning.coordination import (
    fold_range_pairs,
)
from krrood.entity_query_language.verbalization.subquery import (
    aggregation_leaf_attribute,
    aggregation_source_root,
    is_aggregation_subquery,
    is_constrained_query,
    selected_aggregator,
)
from krrood.entity_query_language.verbalization.vocabulary.english import FallbackNouns


class SelectionKind(Enum):
    """The structural shape of a query's selection.

    :cvar ENTITY_SELECTOR: The selected variable is itself an Entity (a sub-query selector).
    :cvar EMPTY: No selected variable — the fallback *"entity"* form.
    :cvar SUBJECT: A plain variable / aggregator selection that can carry restrictions.
    :cvar SET_OF: A :class:`~krrood.entity_query_language.query.query.SetOf` tuple selection.
    """

    ENTITY_SELECTOR = auto()
    EMPTY = auto()
    SUBJECT = auto()
    SET_OF = auto()


@dataclass(frozen=True)
class RestrictionPlan:
    """Partition of a subject's WHERE condition into grouped vs. residual conjuncts."""

    grouped: List[Tuple[Type[RestrictionRule], Any]] = field(default_factory=list)
    """``(rule, folded item)`` pairs that fold into *"whose …"* (the rule renders each)."""

    residual: List[Any] = field(default_factory=list)
    """Folded items (``RangeFold`` or raw expression) for the residual *"such that …"*."""

    @property
    def has_grouped(self) -> bool:
        return bool(self.grouped)

    @property
    def has_residual(self) -> bool:
        return bool(self.residual)


@dataclass(frozen=True)
class GroupPlan:
    """The GROUP BY keys and the expressions aggregated over them."""

    keys: List[Any]
    """The group-by key expressions."""

    aggregated: List[Any]
    """Child expressions aggregated (not group keys) — rendered plural."""

    @property
    def has_keys(self) -> bool:
        return bool(self.keys)


@dataclass(frozen=True)
class OrderPlan:
    """The ORDER BY variable and direction."""

    variable: Any
    descending: bool


@dataclass(frozen=True)
class AggregationValuePlan:
    """A collapsed aggregation value-subquery (*"the maximum amount among …"*)."""

    aggregator: Any
    leaf: Any
    """The leaf :class:`Attribute` of the aggregator chain, or ``None``."""

    is_constrained: bool
    source: Any
    """The chain-root source variable aggregated over, or ``None``."""


@dataclass(frozen=True)
class QueryPlan:
    """Complete *what to say* decomposition of a query (the plan)."""

    kind: SelectionKind
    is_the: bool
    selected_type: str
    subject: Optional[Any]
    subject_restriction: Optional[RestrictionPlan]
    where_condition: Optional[Any]
    group: Optional[GroupPlan]
    having_condition: Optional[Any]
    order: Optional[OrderPlan]
    is_aggregation_subquery: bool
    aggregation_value: Optional[AggregationValuePlan]


@dataclass
class QueryPlanner(Planner[Query, QueryPlan]):
    """Decompose an :class:`Entity` / :class:`SetOf` into a :class:`QueryPlan`."""

    def plan(self) -> QueryPlan:
        self.node.build()
        return QueryPlan(
            kind=self._kind(),
            is_the=self._is_the(),
            selected_type=self._selected_type(),
            subject=self._subject(),
            subject_restriction=self._subject_restriction(),
            where_condition=self._where_condition(),
            group=self._group_plan(),
            having_condition=self._having_condition(),
            order=self._order_plan(),
            is_aggregation_subquery=is_aggregation_subquery(self.node),
            aggregation_value=self._aggregation_value(),
        )

    # ── selection shape ──────────────────────────────────────────────────────

    @property
    def _selected(self):
        return getattr(self.node, "selected_variable", None)

    def _kind(self) -> SelectionKind:
        if isinstance(self.node, SetOf):
            return SelectionKind.SET_OF
        var = self._selected
        if isinstance(var, Entity):
            return SelectionKind.ENTITY_SELECTOR
        if var is None:
            return SelectionKind.EMPTY
        return SelectionKind.SUBJECT

    def _is_the(self) -> bool:
        builder = getattr(self.node, "_quantifier_builder_", None)
        return builder is not None and builder.type is The

    def _selected_type(self) -> str:
        var = self._selected
        if var is not None and getattr(var, "_type_", None):
            return var._type_.__name__
        return FallbackNouns.ENTITY.text

    # ── subject restriction (WHERE partition) ────────────────────────────────

    def _subject(self) -> Optional[Any]:
        if not isinstance(self.node, Entity):
            return None
        return restriction_subject(self.node, self._selected, None)

    def _subject_restriction(self) -> Optional[RestrictionPlan]:
        condition = self._where_condition()
        subject = self._subject()
        if condition is None or subject is None:
            return None
        return self._partition(subject, condition)

    def _partition(self, subject, condition) -> RestrictionPlan:
        """Fold range pairs, then sort conjuncts into grouped (rule-matched) vs. residual."""
        grouped: List[Tuple[Type[RestrictionRule], Any]] = []
        residual: List[Any] = []
        for item in fold_range_pairs(flatten_operands(condition, AND)):
            rule = match_restriction(item, subject, None)
            if rule is None:
                residual.append(item)
            else:
                grouped.append((rule, item))
        return RestrictionPlan(grouped=grouped, residual=residual)

    # ── clauses ──────────────────────────────────────────────────────────────

    def _where_condition(self) -> Optional[Any]:
        where = getattr(self.node, "_where_expression_", None)
        return where.condition if where is not None else None

    def _having_condition(self) -> Optional[Any]:
        having = getattr(self.node, "_having_expression_", None)
        return having.condition if having is not None else None

    def _order_plan(self) -> Optional[OrderPlan]:
        builder = getattr(self.node, "_ordered_by_builder_", None)
        if builder is None:
            return None
        return OrderPlan(variable=builder.variable, descending=builder.descending)

    def _group_plan(self) -> Optional[GroupPlan]:
        grouped = getattr(self.node, "_grouped_by_expression_", None)
        if grouped is None or not grouped.variables_to_group_by:
            return None
        keys = list(grouped.variables_to_group_by)
        root_ids = self._root_var_ids(keys)
        return GroupPlan(keys=keys, aggregated=self._aggregated_expressions(root_ids))

    @staticmethod
    def _root_var_ids(exprs) -> set:
        ids: set = set()
        for expression in exprs:
            root = chain_root(expression)
            if isinstance(root, Variable):
                ids.add(root._id_)
        return ids

    def _aggregated_expressions(self, group_key_root_ids: set) -> List[Any]:
        """Child expressions that are aggregated (not group keys)."""
        var = self._selected if isinstance(self.node, Entity) else None
        if isinstance(var, InstantiatedVariable):
            return [
                child
                for child in var._child_vars_.values()
                if not (
                    isinstance(chain_root(child), Variable)
                    and chain_root(child)._id_ in group_key_root_ids
                )
            ]
        if isinstance(self.node, Query):
            return [
                variable
                for variable in self.node._selected_variables_
                if variable._id_ not in group_key_root_ids
            ]
        return []

    # ── aggregation value-subquery ───────────────────────────────────────────

    def _aggregation_value(self) -> Optional[AggregationValuePlan]:
        if not is_aggregation_subquery(self.node):
            return None
        return AggregationValuePlan(
            aggregator=selected_aggregator(self.node),
            leaf=aggregation_leaf_attribute(self.node),
            is_constrained=is_constrained_query(self.node),
            source=aggregation_source_root(self.node),
        )

"""
Clause **planners** — pure analysis for the individual query clauses that are
constituents both of a query body and (for GROUP BY / ORDER BY) of a standalone
:class:`~krrood.entity_query_language.core.base_expressions.SymbolicExpression`.

Only GROUP BY carries real analysis (which selected expressions are *aggregated* vs.
group keys); ORDER BY and HAVING are pure surface (their assemblers read the node
directly and have no planner).

Reference: Reiter & Dale (2000) — content/structure determination (microplanning).
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Any, List

from krrood.entity_query_language.core.variable import InstantiatedVariable, Variable
from krrood.entity_query_language.query.operations import GroupedBy
from krrood.entity_query_language.query.query import Entity, Query
from krrood.entity_query_language.verbalization.chain_utils import chain_root
from krrood.entity_query_language.verbalization.grammar.planning.base import Planner


@dataclass(frozen=True)
class GroupPlan:
    """The GROUP BY keys and the expressions aggregated over them."""

    keys: List[Any]
    """The group-by key expressions (empty ⇒ a bare *"grouped"* / no clause)."""

    aggregated: List[Any]
    """Selected expressions aggregated (not group keys) — rendered plural; empty for a
    bare :class:`GroupedBy` node (no selection context)."""

    @property
    def has_keys(self) -> bool:
        return bool(self.keys)


@dataclass
class GroupedByPlanner(Planner[Any, GroupPlan]):
    """
    Decompose the GROUP BY of *node* — either a query (``_grouped_by_expression_`` + its
    selection) or a bare :class:`GroupedBy` node — into a :class:`GroupPlan`.
    """

    def plan(self) -> GroupPlan:
        grouped = self._grouped_by()
        if grouped is None or not grouped.variables_to_group_by:
            return GroupPlan(keys=[], aggregated=[])
        keys = list(grouped.variables_to_group_by)
        return GroupPlan(
            keys=keys, aggregated=self._aggregated(self._root_var_ids(keys))
        )

    def _grouped_by(self):
        if isinstance(self.node, GroupedBy):
            return self.node
        return getattr(self.node, "_grouped_by_expression_", None)

    @staticmethod
    def _root_var_ids(exprs) -> set:
        ids: set = set()
        for expression in exprs:
            root = chain_root(expression)
            if isinstance(root, Variable):
                ids.add(root._id_)
        return ids

    def _aggregated(self, group_key_root_ids: set) -> List[Any]:
        """Selected expressions that are aggregated (not group keys); ``[]`` off a query."""
        var = self.node.selected_variable if isinstance(self.node, Entity) else None
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

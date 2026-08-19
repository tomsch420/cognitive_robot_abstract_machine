"""
Incremental construction of relational candidate rule bodies as EQL queries.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from typing_extensions import Any, List

from krrood.class_diagrams.attribute_introspector import DataclassOnlyIntrospector
from krrood.entity_query_language.core.mapped_variable import CanBehaveLikeAVariable
from krrood.entity_query_language.core.variable import Variable
from krrood.entity_query_language.factories import ConditionType, entity, flat_variable
from krrood.entity_query_language.query.query import Entity
from krrood.entity_query_language.rule_mining.exceptions import (
    IncompatibleVariableTypesError,
    UnknownAttributeError,
)
from krrood.symbol_graph.helpers import get_wrapped_field

# %% candidate rule body


@dataclass(eq=False)
class CandidateRuleBody:
    """
    A relational rule body under incremental construction.

    Mirrors the AMIE/WARMR refinement operators (dangling atom, instantiated atom,
    closing atom) as EQL ``.where()`` conditions accumulated on a query, so the query
    itself is the rule. Every extension method mutates this body in place and returns it,
    matching :meth:`~krrood.entity_query_language.query.query.Query.where`'s own
    mutate-and-return-self convention.

    >>> from dataclasses import dataclass
    >>> from typing_extensions import List
    >>> from krrood.entity_query_language.factories import variable
    >>> @dataclass
    ... class Room:
    ...     name: str
    ...     doors: List[str]
    >>> kitchen = Room("Kitchen", doors=["Front", "Back"])
    >>> pantry = Room("Pantry", doors=[])
    >>> body = CandidateRuleBody(head_variable=variable(Room, domain=[kitchen, pantry]))
    >>> _ = body.constrain_variable_to_value(body.head_variable, kitchen)
    >>> [room.name for room in body.to_query().evaluate()]
    ['Kitchen']
    >>> _ = body.extend_with_related_variable(body.head_variable, "doors")
    >>> [room.name for room in body.to_query().evaluate()]
    ['Kitchen', 'Kitchen']
    """

    head_variable: Variable
    """
    The variable whose instances the rule is about; the query's selected variable.
    """

    open_variables: List[CanBehaveLikeAVariable] = field(default_factory=list)
    """
    Variables introduced by a dangling atom that are not yet connected, by a closing
    atom, to another variable already in the body.
    """

    conditions: List[ConditionType] = field(default_factory=list)
    """
    The conditions accumulated so far by the refinement operators.
    """

    def extend_with_related_variable(
        self, source_variable: CanBehaveLikeAVariable, attribute_name: str
    ) -> CandidateRuleBody:
        """
        Apply the dangling-atom operator: introduce a new open variable reached by
        traversing ``attribute_name`` from ``source_variable``.

        A collection-valued attribute is flattened so one open variable is introduced
        per element rather than for the collection itself. Traversing the attribute is
        itself the atom, so no separate condition is needed to bind it.

        :param source_variable: The variable whose attribute is traversed.
        :param attribute_name: The name of the attribute to traverse.
        :return: This candidate rule body, with the traversal folded in.
        :raises UnknownAttributeError: If ``attribute_name`` is not declared on
            ``source_variable``'s static type.
        """
        owner_type = source_variable._type_
        declared_attribute_names = {
            attribute.public_name
            for attribute in DataclassOnlyIntrospector().discover(owner_type)
        }
        if attribute_name not in declared_attribute_names:
            raise UnknownAttributeError(owner_type, attribute_name)

        attribute = getattr(source_variable, attribute_name)
        wrapped_field = get_wrapped_field(owner_type, attribute_name)
        new_variable = (
            flat_variable(attribute)
            if wrapped_field is not None and wrapped_field.is_container
            else attribute
        )
        self.open_variables.append(new_variable)
        self.conditions.append(new_variable)
        return self

    def constrain_variable_to_value(
        self, variable: CanBehaveLikeAVariable, value: Any
    ) -> CandidateRuleBody:
        """
        Apply the instantiated-atom operator: restrict ``variable`` to a literal value.

        :param variable: The variable to constrain.
        :param value: The value ``variable`` must equal.
        :return: This candidate rule body, with the constraint added.
        """
        self.conditions.append(variable == value)
        return self

    def close_by_equating_variables(
        self,
        variable_a: CanBehaveLikeAVariable,
        variable_b: CanBehaveLikeAVariable,
    ) -> CandidateRuleBody:
        """
        Apply the closing-atom operator: equate two open variables and remove both from
        :attr:`open_variables`.

        :param variable_a: The first variable to equate.
        :param variable_b: The second variable to equate.
        :return: This candidate rule body, with the equality added and both variables
            closed.
        :raises IncompatibleVariableTypesError: If the two variables' static types share
            no common subtype, so the equality could never hold.
        """
        type_a, type_b = variable_a._type_, variable_b._type_
        if not (issubclass(type_a, type_b) or issubclass(type_b, type_a)):
            raise IncompatibleVariableTypesError(type_a, type_b)

        self.open_variables = [
            open_variable
            for open_variable in self.open_variables
            if open_variable is not variable_a and open_variable is not variable_b
        ]
        self.conditions.append(variable_a == variable_b)
        return self

    def to_query(self) -> Entity:
        """
        Build the EQL query this rule body represents.

        :return: An entity query selecting :attr:`head_variable`, filtered by
            :attr:`conditions`. The query is the rule; there is no separate rule
            representation.
        """
        return entity(self.head_variable).where(*self.conditions)

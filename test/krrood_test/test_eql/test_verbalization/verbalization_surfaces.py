"""
Committed verbalization surfaces: the sentence each covered symbolic callable renders.

This is the snapshot for :mod:`test_verbalization_surfaces` -- one
:class:`VerbalizationSurface` per covered symbolic callable, referencing the class
itself (so a rename or removal breaks this import) and its approved sentence. Update an
entry when an intentional wording change makes the surface-match test print a new
sentence.
"""

from __future__ import annotations

from dataclasses import dataclass

from typing_extensions import Tuple, Type

from krrood.entity_query_language.factories import (
    AttributeOwnerClass,
    IsClass,
    IsSubclass,
    NodeChildren,
    NodeDescendants,
    NodeId,
    NodeParents,
    NodeType,
    RuntimeType,
)
from krrood.entity_query_language.predicate import (
    HasType,
    HasTypes,
    Is,
    Length,
    SymbolicCallable,
)
from krrood.entity_query_language.verbalization._example_domain import (
    IsReachable,
    WorksIn,
)
from krrood.inheritance_path_length import InheritancePathLength
from krrood.patterns.role_predicates import IsSameSemanticEntity


@dataclass(frozen=True)
class VerbalizationSurface:
    """
    One symbolic callable and the sentence it verbalizes to.
    """

    callable_class: Type[SymbolicCallable]
    """
    The symbolic function or predicate whose surface this records.
    """

    sentence: str
    """
    The approved sentence it renders with the test's placeholder operands.
    """


SURFACES: Tuple[VerbalizationSurface, ...] = (
    VerbalizationSurface(
        AttributeOwnerClass, "the attribute owner class of an Attribute"
    ),
    VerbalizationSurface(IsClass, "an object is a class"),
    VerbalizationSurface(IsSubclass, "a subclass is a subclass of a parent or parents"),
    VerbalizationSurface(NodeChildren, "the node children of a CanBehaveLikeAVariable"),
    VerbalizationSurface(
        NodeDescendants, "the node descendants of a SymbolicExpression"
    ),
    VerbalizationSurface(NodeId, "the node id of a SymbolicExpression"),
    VerbalizationSurface(NodeParents, "the node parents of a SymbolicExpression"),
    VerbalizationSurface(NodeType, "the node type of a Selectable"),
    VerbalizationSurface(RuntimeType, "the runtime type of an object"),
    VerbalizationSurface(HasType, "a variable is of type Integer"),
    VerbalizationSurface(HasTypes, "a variable is of type Integer or Text"),
    VerbalizationSurface(Is, "a first entity is the same object as a second entity"),
    VerbalizationSurface(Length, "the length of an iterable"),
    VerbalizationSurface(IsReachable, "a location is reachable for a body"),
    VerbalizationSurface(WorksIn, "an employee works in a department"),
    VerbalizationSurface(
        InheritancePathLength,
        "the inheritance path length between a child class and a parent class",
    ),
    VerbalizationSurface(
        IsSameSemanticEntity, "an entity 1 is the same entity as an entity 2"
    ),
)

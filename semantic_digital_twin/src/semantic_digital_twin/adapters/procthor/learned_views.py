from dataclasses import dataclass, field
from typing import List, Self, Type, Dict, Tuple

import tqdm
from sqlalchemy.orm import Session

from krrood.ormatic.utils import get_classes_of_ormatic_interface
import semantic_digital_twin.orm.ormatic_interface
from semantic_digital_twin.orm.ormatic_interface import (
    InsideOfDAO,
    RootedSemanticAnnotationDAO,
)
import numpy as np
from krrood.class_diagrams import ClassDiagram
from krrood.class_diagrams.utils import classes_of_module
from krrood.entity_query_language.symbol_graph import SymbolGraph
from krrood.utils import recursive_subclasses
from random_events.interval import closed, SimpleInterval, Bound
from random_events.product_algebra import SimpleEvent

from random_events.variable import Symbolic, Set, Integer

classes, _, _ = get_classes_of_ormatic_interface(
    semantic_digital_twin.orm.ormatic_interface
)
class_diagram = ClassDiagram(classes)


def create_variable_for_type_hierarchy(
    class_diagram: ClassDiagram,
) -> Tuple[Symbolic, Dict[Type, Set]]:
    """
    Create a symbolic variable for the type hierarchy containing only the leaf types as possible values.
    Additionally, create a dict that maps every type to the leaf types that this can be replaced by.
    """
    inheritance_graph = class_diagram.inheritance_subgraph_without_unreachable_nodes
    leaves = [
        node.clazz
        for node in inheritance_graph.nodes()
        if inheritance_graph.out_degree(node.index) == 0
    ]

    class_variable = Symbolic("type", Set.from_iterable(leaves))

    possible_sets = {}
    for node in inheritance_graph.nodes():
        current_possible_sets = []
        for leaf in leaves:
            if issubclass(leaf, node.clazz):
                current_possible_sets.append(leaf)
        if len(current_possible_sets) > 0:
            possible_sets[node.clazz] = class_variable.make_value(current_possible_sets)
    return class_variable, possible_sets


@dataclass
class AnnotatedInsideOfView:
    """
    An enrichment to the InsideOfDAO that also contains the semantic annotations of the two sides of the relation.
    This is aggregated from a database.
    """

    inside_of_dao: InsideOfDAO
    """
    The raw inside of relation.
    """

    body_side_semantic_annotation: List[RootedSemanticAnnotationDAO] = field(
        default_factory=list
    )
    """
    The annotations that involve the `self.inside_of_dao.body` side of the relation.
    """

    other_side_semantic_annotation: List[RootedSemanticAnnotationDAO] = field(
        default_factory=list
    )
    """
    The annotations that involve the `self.inside_of_dao.other` side of the relation.
    """

    @classmethod
    def from_database(cls, session: Session) -> List[Self]:
        """
        :param session: The SQLAlchemy session to query the database
        :return: a list of AnnotatedInsideOfDAO objects from the database.
        """
        inside_ofs = session.query(InsideOfDAO).all()
        all_annotations = session.query(RootedSemanticAnnotationDAO).all()

        results = []
        for inside_of in tqdm.tqdm(inside_ofs):
            body_annotations = [
                ann for ann in all_annotations if ann.root_id == inside_of.body_id
            ]
            other_annotations = [
                ann for ann in all_annotations if ann.root_id == inside_of.other_id
            ]
            results.append(
                cls(
                    inside_of_dao=inside_of,
                    body_side_semantic_annotation=body_annotations,
                    other_side_semantic_annotation=other_annotations,
                )
            )
        return results


@dataclass
class SemanticInsideOfView:

    body_type: Type
    other_type: Type
    containment_ratio: float

    @classmethod
    def from_annotated_inside_of_view(cls, view: AnnotatedInsideOfView) -> Self: ...

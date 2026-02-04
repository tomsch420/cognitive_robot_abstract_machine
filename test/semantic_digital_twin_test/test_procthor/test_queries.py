import numpy as np
from sqlalchemy import select

from semantic_digital_twin.adapters.procthor.learned_views import (
    AnnotatedInsideOfView,
    create_variable_for_type_hierarchy,
    SemanticInsideOfView,
)
from semantic_digital_twin.adapters.procthor.procthor_parser import (
    procthor_sessionmaker,
)
from semantic_digital_twin.orm.ormatic_interface import InsideOfDAO
from semantic_digital_twin.reasoning.predicates import InsideOf


def test_learning_workflow():

    session = procthor_sessionmaker()()

    data = AnnotatedInsideOfView.from_database(session)
    aggregate = np.array(
        [
            [
                len(d.body_side_semantic_annotation),
                len(d.other_side_semantic_annotation),
            ]
            for d in data
        ]
    )

    d = [
        d
        for d in data
        if len(d.body_side_semantic_annotation) > 0
        and len(d.other_side_semantic_annotation) > 0
    ]

    a = SemanticInsideOfView.from_annotated_inside_of_view(d[0])

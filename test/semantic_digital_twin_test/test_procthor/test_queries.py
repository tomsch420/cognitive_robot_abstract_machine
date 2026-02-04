import json

import numpy as np
import pandas as pd
import plotly.graph_objs as go

from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe

from probabilistic_model.learning.jpt.jpt import JPT
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
from semantic_digital_twin.semantic_annotations.semantic_annotations import (
    Floor,
    Wall,
    Door,
)


def test_learning_workflow():

    session = procthor_sessionmaker()()

    data = AnnotatedInsideOfView.from_database(session)

    filtered_data = [
        d
        for d in data
        if len(d.body_side_semantic_annotation) > 0
        and len(d.other_side_semantic_annotation) > 0
    ]

    semantic_inside_of_views = SemanticInsideOfView.from_annotated_inside_of_views(
        filtered_data
    )

    filtered_views = [
        s
        for s in semantic_inside_of_views
        if not issubclass(s.body_type, (Floor, Wall, Door))
        and not issubclass(s.other_type, (Floor, Wall, Door))
    ]
    dataset = pd.DataFrame(
        [
            [f.body_type.__name__, f.other_type.__name__, f.containment_ratio]
            for f in filtered_views
        ],
        columns=["body_type", "other_type", "containment_ratio"],
    )

    variables = infer_variables_from_dataframe(dataset, scale_continuous_types=False)
    model = JPT(variables, min_samples_leaf=10)
    pc = model.fit(dataset)

    print(variables)
    for v in variables:
        m = pc.marginal([v])
        fig = go.Figure(m.plot())
        fig.show()

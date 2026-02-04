from sqlalchemy import select

from semantic_digital_twin.adapters.procthor.learned_views import AnnotatedInsideOfView
from semantic_digital_twin.adapters.procthor.procthor_parser import (
    procthor_sessionmaker,
)
from semantic_digital_twin.orm.ormatic_interface import InsideOfDAO
from semantic_digital_twin.reasoning.predicates import InsideOf


def test_getting_all_inside_of_relations():

    session = procthor_sessionmaker()()

    data = AnnotatedInsideOfView.from_database(session)
    # print(data)

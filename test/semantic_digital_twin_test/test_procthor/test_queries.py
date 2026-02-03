from sqlalchemy import select

from semantic_digital_twin.adapters.procthor.procthor_parser import procthor_sessionmaker
from semantic_digital_twin.orm.ormatic_interface import InsideOfDAO
from semantic_digital_twin.reasoning.predicates import InsideOf


def test_getting_all_inside_of_relations():

    session = procthor_sessionmaker()()

    statement = select(InsideOfDAO).limit(50)
    result = session.scalars(statement).all()


    print(result)
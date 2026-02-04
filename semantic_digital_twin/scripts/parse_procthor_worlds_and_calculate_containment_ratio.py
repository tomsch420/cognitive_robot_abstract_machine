import itertools
import logging
import os

import prior
import tqdm
from sqlalchemy.orm import Session

import semantic_digital_twin.adapters.procthor.procthor_resolver
from krrood.entity_query_language.symbol_graph import SymbolGraph
from krrood.ormatic.dao import to_dao, ToDataAccessObjectState
from krrood.ormatic.utils import classes_of_module, create_engine, drop_database
from semantic_digital_twin.adapters.procthor.procthor_parser import (
    ProcTHORParser,
    procthor_sessionmaker,
)
from semantic_digital_twin.adapters.procthor.procthor_resolver import (
    ProcthorResolver,
)
from semantic_digital_twin.orm.ormatic_interface import *
from semantic_digital_twin.orm.utils import semantic_digital_twin_sessionmaker
from semantic_digital_twin.reasoning.predicates import InsideOf
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation


def parse_procthor_worlds_and_calculate_containment_ratio(
    world_parsing_start_index: int = 0, drop_existing_procthor_database: bool = True
):
    semantic_world_session = semantic_digital_twin_sessionmaker()()
    procthor_experiments_session = procthor_sessionmaker()()

    if drop_existing_procthor_database:
        drop_database(procthor_experiments_session.bind)
        Base.metadata.create_all(procthor_experiments_session.bind)

    dataset = prior.load_dataset("procthor-10k")

    # Iterate through all JSON files in the directory
    for index, house in enumerate(
        tqdm.tqdm(dataset["train"], desc="Parsing Procthor worlds")
    ):
        if index < world_parsing_start_index:
            continue
        try:
            parser = ProcTHORParser(f"house_{index}", house, semantic_world_session)
            world = parser.parse()
        except Exception as e:
            logging.error(f"Error parsing house {index}: {e}")
            continue
        # resolve views
        resolver = ProcthorResolver(
            [
                cls
                for cls in classes_of_module(
                    semantic_digital_twin.adapters.procthor.procthor_resolver
                )
                if issubclass(cls, SemanticAnnotation)
            ]
        )
        for body in world.bodies:
            resolved = resolver.resolve(body.name.name)
            if resolved:
                with world.modify_world():
                    world.add_semantic_annotation(
                        resolved(body=body), skip_duplicates=True
                    )

        state = ToDataAccessObjectState()
        daos = []

        world_dao = to_dao(world, state=state)
        procthor_experiments_session.add(world_dao)

        for kse, other in itertools.product(
            world.kinematic_structure_entities, world.kinematic_structure_entities
        ):
            if kse != other:
                is_inside = InsideOf(kse, other)
                if is_inside() > 0.0:
                    dao = to_dao(is_inside, state=state)
                    daos.append(dao)

        procthor_experiments_session.add_all(daos)
        procthor_experiments_session.commit()
        procthor_experiments_session.expunge_all()
        semantic_world_session.expunge_all()
        SymbolGraph().clear()


if __name__ == "__main__":
    parse_procthor_worlds_and_calculate_containment_ratio(
        world_parsing_start_index=0, drop_existing_procthor_database=True
    )

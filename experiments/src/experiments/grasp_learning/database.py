"""
Storing grasp attempts and learned grasp models in a database, and reading them back.
"""

from __future__ import annotations

import os
from collections import defaultdict
from dataclasses import dataclass
from enum import StrEnum

import numpy as np
from sqlalchemy import func, select
from sqlalchemy.orm import Session
from typing_extensions import Dict, Iterable, List, Set

from experiments.grasp_learning.models import GraspModel, GraspModelLibrary
from experiments.grasp_learning.records import GraspAttempt, GraspLearningTask
from experiments.physical_pick_up.objects import GraspableRoboCasaCategory
from experiments.orm.ormatic_interface import (
    Base,
    GraspAttemptDAO,
    GraspLearningTaskDAO,
    GraspModelDAO,
)
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.ormatic.utils import create_engine

# %% connecting


class GraspDatabaseEnvironmentVariable(StrEnum):
    """
    The environment variables the grasp database is configured by.
    """

    URI = "GRASP_LEARNING_DATABASE_URI"
    """
    The URI of the database, for example
    ``postgresql+psycopg://user:password@host:5432/grasp_learning``.
    """


# %% the database


@dataclass
class GraspDatabase:
    """
    The grasp attempts and the grasp models of all tasks.
    """

    session: Session
    """
    The session the database is used through.
    """

    @classmethod
    def from_environment(cls) -> GraspDatabase:
        """
        :return: The database :attr:`GraspDatabaseEnvironmentVariable.URI` names, with
            its tables created if they are missing.
        """
        return cls.connect(os.environ[GraspDatabaseEnvironmentVariable.URI])

    @classmethod
    def connect(cls, uri: str) -> GraspDatabase:
        """
        :param uri: The URI of the database.
        :return: The database, with its tables created if they are missing.
        """
        engine = create_engine(uri)
        Base.metadata.create_all(engine)
        return cls(session=Session(engine))

    def add_attempts(self, attempts: Iterable[GraspAttempt]) -> None:
        """
        Store attempts.

        :param attempts: The attempts to store.
        """
        self.session.add_all([to_dao(attempt) for attempt in attempts])
        self.session.commit()

    def attempts(self, task: GraspLearningTask) -> List[GraspAttempt]:
        """
        :param task: The task to read the attempts of.
        :return: Every stored attempt of that task, in the order they were stored.
        """
        return list(self.numbered_attempts(task).values())

    def numbered_attempts(self, task: GraspLearningTask) -> Dict[int, GraspAttempt]:
        """
        :param task: The task to read the attempts of.
        :return: Every stored attempt of that task by the number it was stored under,
            in the order they were stored.
        """
        self.session.expire_all()
        query = (
            select(GraspAttemptDAO)
            .join(GraspAttemptDAO.task)
            .where(
                GraspLearningTaskDAO.annotation_type == task.annotation_type,
                GraspLearningTaskDAO.grasped_part == task.grasped_part,
                GraspLearningTaskDAO.gripper == task.gripper,
            )
            .order_by(GraspAttemptDAO.database_id)
        )
        return {dao.database_id: dao.from_dao() for dao in self.session.scalars(query)}

    def attempts_on_category(
        self, category: GraspableRoboCasaCategory
    ) -> List[GraspAttempt]:
        """
        :param category: A category of RoboCasa objects.
        :return: Every stored attempt on the category's objects, in the order they were
            stored.
        """
        self.session.expire_all()
        object_names = [
            object_name
            for object_name in self.session.scalars(
                select(GraspAttemptDAO.object_name).distinct()
            )
            if category.is_body_name_of(object_name)
        ]
        query = (
            select(GraspAttemptDAO)
            .where(GraspAttemptDAO.object_name.in_(object_names))
            .order_by(GraspAttemptDAO.database_id)
        )
        return [dao.from_dao() for dao in self.session.scalars(query)]

    def sampled_attempts(
        self,
        excluded_object_names: Set[str],
        attempts_per_object: int,
        generator: np.random.Generator,
        numbers_per_query: int = 10000,
    ) -> List[GraspAttempt]:
        """
        :param excluded_object_names: The objects whose attempts are left out.
        :param attempts_per_object: The most attempts drawn on each object.
        :param generator: Draws the attempts.
        :param numbers_per_query: How many attempts are read in one query, which keeps
            each query within the database's limit on parameters.
        :return: Attempts of any task drawn without replacement on every other
            object, in the order they were stored.
        """
        numbers_by_object = defaultdict(list)
        for number, object_name in self.session.execute(
            select(GraspAttemptDAO.database_id, GraspAttemptDAO.object_name)
        ):
            if object_name not in excluded_object_names:
                numbers_by_object[object_name].append(number)
        chosen = sorted(
            number
            for _, numbers in sorted(numbers_by_object.items())
            for number in generator.choice(
                numbers, min(len(numbers), attempts_per_object), replace=False
            ).tolist()
        )
        attempts = []
        for start in range(0, len(chosen), numbers_per_query):
            query = (
                select(GraspAttemptDAO)
                .where(
                    GraspAttemptDAO.database_id.in_(
                        chosen[start : start + numbers_per_query]
                    )
                )
                .order_by(GraspAttemptDAO.database_id)
            )
            attempts.extend(dao.from_dao() for dao in self.session.scalars(query))
        return attempts

    def last_attempt_number(self) -> int:
        """
        :return: The number the latest attempt of any task was stored under; 0 if none
            is stored.
        """
        self.session.expire_all()
        query = select(func.max(GraspAttemptDAO.database_id))
        return self.session.scalar(query) or 0

    def add_model(self, model: GraspModel) -> None:
        """
        Store a model.

        :param model: The model to store.
        """
        self.session.add(to_dao(model))
        self.session.commit()

    def model_library(self) -> GraspModelLibrary:
        """
        :return: Every stored model, the latest of each task last.
        """
        query = select(GraspModelDAO).order_by(GraspModelDAO.database_id)
        return GraspModelLibrary(
            models=[dao.from_dao() for dao in self.session.scalars(query)]
        )


# %% the command line


def main() -> None:
    """
    Create the tables of the database
    :attr:`GraspDatabaseEnvironmentVariable.URI` names, once, before several workers
    use it at the same time.
    """
    GraspDatabase.from_environment()


if __name__ == "__main__":
    main()

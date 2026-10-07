"""
Storing grasp attempts and learned grasp models in a database, and reading them back.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from enum import StrEnum

from sqlalchemy import select
from sqlalchemy.orm import Session
from typing_extensions import Iterable, List

from experiments.grasp_learning.models import GraspModel, GraspModelLibrary
from experiments.grasp_learning.records import GraspAttempt, GraspLearningTask
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
        return [dao.from_dao() for dao in self.session.scalars(query)]

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

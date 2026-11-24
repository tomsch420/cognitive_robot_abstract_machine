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

from experiments.grasp_learning.exceptions import (
    NoPickUpTrial,
    NoRecordedGraspAttempt,
    NoStoredWorld,
)
from experiments.grasp_learning.models import GraspModel, GraspModelLibrary
from experiments.grasp_learning.records import (
    GraspAttempt,
    GraspLearningTask,
    PickUpTrial,
    RecordedGraspAttempt,
    ReleaseTrial,
)
from experiments.physical_pick_up.objects import GraspableRoboCasaCategory
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.ormatic.utils import create_engine
from semantic_digital_twin.world import World

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
        from experiments.orm.ormatic_interface import Base

        engine = create_engine(uri)
        Base.metadata.create_all(engine)
        return cls(session=Session(engine))

    def add_attempts(
        self, attempts: Iterable[GraspAttempt | RecordedGraspAttempt]
    ) -> None:
        """
        Store attempts; the attempt of a :class:`RecordedGraspAttempt` counts among the
        attempts of its task like one stored alone.

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

    def recorded_attempt(self, database_id: int) -> RecordedGraspAttempt:
        """
        :param database_id: The number an attempt with its run was stored under.
        :return: The attempt with its run.
        :raises NoRecordedGraspAttempt: If none is stored under that number.
        """
        from experiments.orm.ormatic_interface import RecordedGraspAttemptDAO

        stored = self.session.get(RecordedGraspAttemptDAO, database_id)
        if stored is None:
            raise NoRecordedGraspAttempt(database_id=database_id)
        return stored.from_dao()

    def starting_world(self, *, database_id: int) -> World:
        """
        :param database_id: The number a world was stored under.
        :return: The world, as it was stored.
        :raises NoStoredWorld: If none is stored under that number.
        """
        from semantic_digital_twin.orm.ormatic_interface import WorldMappingDAO

        stored = self.session.get(WorldMappingDAO, database_id)
        if stored is None:
            raise NoStoredWorld(database_id=database_id)
        return stored.from_dao()

    def add_trial(self, *, trial: PickUpTrial | ReleaseTrial) -> None:
        """
        Store a trial with its run.

        :param trial: The trial to store.
        """
        self.session.add(to_dao(trial))
        self.session.commit()

    def pick_up_trial(self, *, database_id: int) -> PickUpTrial:
        """
        :param database_id: The number a pick-up trial was stored under.
        :return: The trial with its run.
        :raises NoPickUpTrial: If none is stored under that number.
        """
        from experiments.orm.ormatic_interface import PickUpTrialDAO

        stored = self.session.get(PickUpTrialDAO, database_id)
        if stored is None:
            raise NoPickUpTrial(database_id=database_id)
        return stored.from_dao()

    def numbered_attempts(self, task: GraspLearningTask) -> Dict[int, GraspAttempt]:
        """
        :param task: The task to read the attempts of.
        :return: Every stored attempt of that task by the number it was stored under,
            in the order they were stored.
        """
        from experiments.orm.ormatic_interface import (
            GraspAttemptDAO,
            GraspLearningTaskDAO,
        )

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
        from experiments.orm.ormatic_interface import GraspAttemptDAO

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
        from experiments.orm.ormatic_interface import GraspAttemptDAO

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
        from experiments.orm.ormatic_interface import GraspAttemptDAO

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
        from experiments.orm.ormatic_interface import GraspModelDAO

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

"""
The database the runs of the atomic tasks are stored in.
"""

from __future__ import annotations

from dataclasses import dataclass

from sqlalchemy import select
from sqlalchemy.orm import Session

from experiments.orm.ormatic_interface import Base, StoredTaskRunDAO
from experiments.robocasa_atomic_tasks.exceptions import NoStoredRun
from experiments.robocasa_atomic_tasks.stored_runs import StoredTaskRun
from krrood.ormatic.data_access_objects.helper import to_dao
from krrood.ormatic.utils import create_engine

# %% the database


@dataclass
class TaskRunDatabase:
    """
    The stored runs of the atomic tasks.
    """

    session: Session
    """
    The session the database is used through.
    """

    @classmethod
    def connect(cls, uri: str) -> TaskRunDatabase:
        """
        :param uri: The URI of the database.
        :return: The database, with its tables created if they are missing.
        """
        engine = create_engine(uri)
        Base.metadata.create_all(engine)
        return cls(session=Session(engine))

    def add(self, run: StoredTaskRun) -> None:
        """
        Store a run.

        :param run: The run.
        """
        self.session.add(to_dao(run))
        self.session.commit()

    def latest_run_of(self, task: str) -> StoredTaskRun:
        """
        :param task: The name of a task.
        :return: The run of the task stored last.
        :raises NoStoredRun: If no run of the task is stored.
        """
        query = (
            select(StoredTaskRunDAO)
            .where(StoredTaskRunDAO.task == task)
            .order_by(StoredTaskRunDAO.database_id.desc())
            .limit(1)
        )
        stored = self.session.scalars(query).first()
        if stored is None:
            raise NoStoredRun(task=task)
        return stored.from_dao()

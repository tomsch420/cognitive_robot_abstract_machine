"""
Reporting the grasp models stored in the database: for the latest model of every task,
how many attempts it was learned from, how often those lifted the object, and how often
the grasps drawn from it did.

Run it with ``python -m experiments.grasp_learning.report``; the database is the one
:attr:`~experiments.grasp_learning.database.GraspDatabaseEnvironmentVariable.URI` names.
"""

from __future__ import annotations

from typing_extensions import Dict, List, Tuple

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.models import GraspModel


def latest_models(models: List[GraspModel]) -> List[GraspModel]:
    """
    :param models: Models of any tasks, the latest of each task last.
    :return: The latest model of every task, ordered by the task's gripper, object and
        grasped part.
    """
    latest: Dict[Tuple[str, str, str], GraspModel] = {}
    for model in models:
        task = model.task
        key = (
            task.gripper.__name__,
            task.annotation_type.__name__,
            task.grasped_part.__name__,
        )
        latest[key] = model
    return [latest[key] for key in sorted(latest)]


def report(models: List[GraspModel]) -> str:
    """
    :param models: The models to report, one row each.
    :return: A Markdown table of the models.
    """
    rows = [
        "| Gripper | Object | Grasped at | Attempts | Lifted | Lifted with the model's "
        "grasps |",
        "|---|---|---|---|---|---|",
    ]
    for model in models:
        task = model.task
        verified = (
            "not verified"
            if model.verified_lift_rate is None
            else f"{model.verified_lift_rate:.0%}"
        )
        rows.append(
            f"| {task.gripper.__name__} | {task.annotation_type.__name__} | "
            f"{task.grasped_part.__name__} | {model.number_of_attempts} | "
            f"{model.lift_rate_of_attempts:.0%} | {verified} |"
        )
    return "\n".join(rows)


def main() -> None:
    """
    Print the report of the latest model of every task in the database.
    """
    database = GraspDatabase.from_environment()
    print(report(latest_models(database.model_library().models)))


if __name__ == "__main__":
    main()

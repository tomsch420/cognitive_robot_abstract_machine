"""
Learning grasp models for every graspable RoboCasa object, with many simulations trying
grasps at once.

Every model of every :class:`~experiments.physical_pick_up.objects.GraspableRoboCasaObject`
first has grasps tried and stored, one process per model. Then a model is learned per
category from every stored attempt of its task and verified on the category's first
model, again one process per category. The processes run the grasp learning pipeline and
share its database.

Run it with ``python -m experiments.grasp_learning.robocasa``; see ``--help``.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path

from typing_extensions import Dict, List

from experiments.physical_pick_up.objects import GraspableRoboCasaObject
from experiments.physical_pick_up.robots import PickUpRobot
from semantic_digital_twin.adapters.robocasa_dataset.loader import (
    RoboCasaDatasetLoader,
)

# %% one model of a graspable object


@dataclass(frozen=True)
class RoboCasaModel:
    """
    One downloaded model of a graspable RoboCasa object.
    """

    graspable: GraspableRoboCasaObject
    """
    The object the model is one of.
    """

    instance_index: int
    """
    Which of the category's models it is.
    """

    @property
    def name(self) -> str:
        """
        The name of the model, as the logs of its processes are named.
        """
        return f"{self.graspable.name.lower()}_{self.instance_index}"

    def pipeline_arguments(self) -> List[str]:
        """
        :return: The arguments choosing this model on the command line of the grasp
            learning pipeline.
        """
        return [
            "--robocasa-object",
            self.graspable.name.lower(),
            "--instance",
            str(self.instance_index),
        ]


# %% running the pipeline in many processes


class ThreadCountVariable(StrEnum):
    """
    The environment variables that limit how many threads the numerical libraries of a
    process start.
    """

    OPENMP = "OMP_NUM_THREADS"
    OPENBLAS = "OPENBLAS_NUM_THREADS"
    MKL = "MKL_NUM_THREADS"


@dataclass
class ParallelRoboCasaGraspLearning:
    """
    Learns a grasp model for every graspable RoboCasa object, running the grasp learning
    pipeline in many processes at once.
    """

    stored_parts: Path
    """
    The parts located before for the objects; an object without stored parts is grasped
    as a whole.
    """

    log_directory: Path
    """
    Where every process writes its output, one file each.
    """

    processes: int = 32
    """
    How many processes run at once.
    """

    attempts_per_model: int = 20
    """
    How many grasps drawn from the stated regions are tried on each model.
    """

    verifying_attempts: int = 20
    """
    How many grasps drawn from the learned model are tried for each category.
    """

    robot: PickUpRobot = PickUpRobot.PR2
    """
    The robot that picks the objects up.
    """

    threads_per_process: int = 2
    """
    How many threads the numerical libraries of each process may start.
    """

    graspables: List[GraspableRoboCasaObject] = field(
        default_factory=lambda: list(GraspableRoboCasaObject)
    )
    """
    The objects models are learned for.
    """

    loader: RoboCasaDatasetLoader = field(default_factory=RoboCasaDatasetLoader)
    """
    Finds the downloaded models.
    """

    def models(self) -> List[RoboCasaModel]:
        """
        :return: Every downloaded model of every object of :attr:`graspables`.
        """
        return [
            RoboCasaModel(graspable=graspable, instance_index=instance_index)
            for graspable in self.graspables
            for instance_index in range(
                len(self.loader.object_model_files(graspable.value.category))
            )
        ]

    def run(self) -> Dict[str, str]:
        """
        Try grasps on every model, then learn and verify a model for every category.

        :return: The last line each verifying process printed, by the name of the model
            it verified on.
        """
        models = self.models()
        self.log_directory.mkdir(parents=True, exist_ok=True)
        self._run_all(
            {
                f"attempts_{model.name}": [
                    *model.pipeline_arguments(),
                    "--attempts",
                    str(self.attempts_per_model),
                    "--seed",
                    str(seed),
                    "--attempts-only",
                ]
                for seed, model in enumerate(models)
            }
        )
        first_models = [model for model in models if model.instance_index == 0]
        return self._run_all(
            {
                f"model_{model.name}": [
                    *model.pipeline_arguments(),
                    "--attempts",
                    "0",
                    "--verifying-attempts",
                    str(self.verifying_attempts),
                ]
                for model in first_models
            }
        )

    def _run_all(self, arguments_by_name: Dict[str, List[str]]) -> Dict[str, str]:
        """
        Run the pipeline once per entry, :attr:`processes` at a time.

        :param arguments_by_name: The pipeline's arguments, by the name its log is
            written under.
        :return: The last line each run printed, by its name; an empty line for a run
            that printed nothing.
        """
        with ThreadPoolExecutor(max_workers=self.processes) as executor:
            last_lines = executor.map(
                self._run_pipeline, arguments_by_name, arguments_by_name.values()
            )
            return dict(zip(arguments_by_name, last_lines))

    def _run_pipeline(self, name: str, arguments: List[str]) -> str:
        """
        Run the pipeline in a process of its own, writing its output to its log.

        :param name: The name the log is written under.
        :param arguments: The pipeline's arguments.
        :return: The last line the run printed.
        """
        log_file = self.log_directory / f"{name}.log"
        environment = dict(os.environ)
        environment.update(
            {
                variable.value: str(self.threads_per_process)
                for variable in ThreadCountVariable
            }
        )
        with log_file.open("w") as log:
            subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "experiments.grasp_learning.pipeline",
                    "--robot",
                    self.robot.name.lower(),
                    "--stored-parts",
                    str(self.stored_parts),
                    *arguments,
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=environment,
            )
        lines = log_file.read_text().splitlines()
        return lines[-1] if lines else ""


# %% the command line


def main() -> None:
    """
    Learn grasp models for every graspable RoboCasa object and print how each category's
    model did.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stored-parts",
        type=Path,
        required=True,
        help="the parts located before for the objects",
    )
    parser.add_argument(
        "--log-directory", type=Path, required=True, help="where the logs go"
    )
    parser.add_argument(
        "--processes", type=int, default=ParallelRoboCasaGraspLearning.processes
    )
    parser.add_argument(
        "--attempts-per-model",
        type=int,
        default=ParallelRoboCasaGraspLearning.attempts_per_model,
    )
    parser.add_argument(
        "--verifying-attempts",
        type=int,
        default=ParallelRoboCasaGraspLearning.verifying_attempts,
    )
    parser.add_argument(
        "--robot",
        choices=[robot.name.lower() for robot in PickUpRobot],
        default=ParallelRoboCasaGraspLearning.robot.name.lower(),
    )
    parser.add_argument(
        "--objects",
        nargs="+",
        choices=[graspable.name.lower() for graspable in GraspableRoboCasaObject],
        default=[graspable.name.lower() for graspable in GraspableRoboCasaObject],
        help="the objects to learn models for; all of them by default",
    )
    arguments = parser.parse_args()
    learning = ParallelRoboCasaGraspLearning(
        stored_parts=arguments.stored_parts,
        log_directory=arguments.log_directory,
        processes=arguments.processes,
        attempts_per_model=arguments.attempts_per_model,
        verifying_attempts=arguments.verifying_attempts,
        robot=PickUpRobot[arguments.robot.upper()],
        graspables=[
            GraspableRoboCasaObject[name.upper()] for name in arguments.objects
        ],
    )
    for name, last_line in learning.run().items():
        print(f"{name}: {last_line}")


if __name__ == "__main__":
    main()

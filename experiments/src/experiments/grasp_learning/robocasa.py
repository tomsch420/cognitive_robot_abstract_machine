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
from abc import ABC, abstractmethod
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


class GraspLearningModule(StrEnum):
    """
    The command lines the parallel runs start processes of.
    """

    PIPELINE = "experiments.grasp_learning.pipeline"
    """
    The grasp learning pipeline, for one model.
    """

    POLICY_ITERATION = "experiments.grasp_learning.policy_iteration"
    """
    The policy iteration, for every model of one object.
    """


@dataclass
class ParallelRoboCasaRuns(ABC):
    """
    Runs grasp learning for the graspable RoboCasa objects in many processes at once,
    each with its output in a log of its own.
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

    @abstractmethod
    def run(self) -> Dict[str, str]:
        """
        Learn the models.

        :return: The last line each process that reports a model printed, by the name
            of its log.
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

    def run_all(
        self, module: GraspLearningModule, arguments_by_name: Dict[str, List[str]]
    ) -> Dict[str, str]:
        """
        Run a module once per entry, :attr:`processes` at a time.

        :param module: The module to run.
        :param arguments_by_name: Its arguments, by the name its log is written under.
        :return: The last line each run printed, by its name; an empty line for a run
            that printed nothing.
        """
        self.log_directory.mkdir(parents=True, exist_ok=True)
        with ThreadPoolExecutor(max_workers=self.processes) as executor:
            last_lines = executor.map(
                lambda name, arguments: self.run_one(module, name, arguments),
                arguments_by_name,
                arguments_by_name.values(),
            )
            return dict(zip(arguments_by_name, last_lines))

    def run_one(
        self, module: GraspLearningModule, name: str, arguments: List[str]
    ) -> str:
        """
        Run a module in a process of its own, writing its output to its log.

        :param module: The module to run.
        :param name: The name the log is written under.
        :param arguments: The module's arguments.
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
                    module.value,
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


@dataclass
class ParallelRoboCasaGraspLearning(ParallelRoboCasaRuns):
    """
    Tries grasps on every model, one process per model, then learns a model per category
    from every stored attempt of its task and verifies it on the category's first model.
    """

    attempts_per_model: int = 20
    """
    How many grasps drawn from the stated regions are tried on each model.
    """

    verifying_attempts: int = 20
    """
    How many grasps drawn from the learned model are tried for each category.
    """

    def run(self) -> Dict[str, str]:
        models = self.models()
        self.run_all(
            GraspLearningModule.PIPELINE,
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
            },
        )
        first_models = [model for model in models if model.instance_index == 0]
        return self.run_all(
            GraspLearningModule.PIPELINE,
            {
                f"model_{model.name}": [
                    *model.pipeline_arguments(),
                    "--attempts",
                    "0",
                    "--verifying-attempts",
                    str(self.verifying_attempts),
                ]
                for model in first_models
            },
        )


@dataclass
class ParallelRoboCasaPolicyIteration(ParallelRoboCasaRuns):
    """
    Iterates the grasp policy of every graspable object, one iteration per object over
    all of its models, each with workers of its own; :attr:`processes` iterations run at
    once.
    """

    workers_per_iteration: int = 8
    """
    How many workers try the grasps of one iteration.
    """

    results: Path = Path("policy_iteration")
    """
    Where each object's iteration writes its rounds, in a directory named after it.
    """

    iteration_arguments: List[str] = field(default_factory=list)
    """
    Further arguments of every iteration, such as its stopping rule.
    """

    def run(self) -> Dict[str, str]:
        return self.run_all(
            GraspLearningModule.POLICY_ITERATION,
            {
                f"iteration_{graspable.name.lower()}": self.arguments_of(graspable)
                for graspable in self.graspables
            },
        )

    def arguments_of(self, graspable: GraspableRoboCasaObject) -> List[str]:
        """
        :param graspable: One of :attr:`graspables`.
        :return: The arguments of its iteration.
        """
        return [
            "--robocasa-object",
            graspable.name.lower(),
            "--workers",
            str(self.workers_per_iteration),
            "--results",
            str(self.results / graspable.name.lower()),
            *self.iteration_arguments,
        ]


# %% the command line


class RoboCasaRun(StrEnum):
    """
    The runs the command line offers.
    """

    LEARN = "learn"
    """
    Try grasps on every model, then learn once.
    """

    ITERATE = "iterate"
    """
    Iterate the policy of every object.
    """


def main() -> None:
    """
    Learn grasp models for the graspable RoboCasa objects and print how each one did.
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
    parser.add_argument("--processes", type=int, default=ParallelRoboCasaRuns.processes)
    parser.add_argument(
        "--robot",
        choices=[robot.name.lower() for robot in PickUpRobot],
        default=ParallelRoboCasaRuns.robot.name.lower(),
    )
    parser.add_argument(
        "--objects",
        nargs="+",
        choices=[graspable.name.lower() for graspable in GraspableRoboCasaObject],
        default=[graspable.name.lower() for graspable in GraspableRoboCasaObject],
        help="the objects to learn models for; all of them by default",
    )
    runs = parser.add_subparsers(dest="run", required=True)
    once = runs.add_parser(
        RoboCasaRun.LEARN, help="try grasps on every model, then learn once"
    )
    once.add_argument(
        "--attempts-per-model",
        type=int,
        default=ParallelRoboCasaGraspLearning.attempts_per_model,
    )
    once.add_argument(
        "--verifying-attempts",
        type=int,
        default=ParallelRoboCasaGraspLearning.verifying_attempts,
    )
    iterate = runs.add_parser(
        RoboCasaRun.ITERATE,
        help="iterate the policy of every object; further arguments go to every "
        "iteration",
    )
    iterate.add_argument(
        "--workers-per-iteration",
        type=int,
        default=ParallelRoboCasaPolicyIteration.workers_per_iteration,
    )
    iterate.add_argument("--results", type=Path, required=True)
    arguments, iteration_arguments = parser.parse_known_args()
    common = dict(
        stored_parts=arguments.stored_parts,
        log_directory=arguments.log_directory,
        processes=arguments.processes,
        robot=PickUpRobot[arguments.robot.upper()],
        graspables=[
            GraspableRoboCasaObject[name.upper()] for name in arguments.objects
        ],
    )
    if arguments.run == RoboCasaRun.LEARN:
        runs_of_objects = ParallelRoboCasaGraspLearning(
            **common,
            attempts_per_model=arguments.attempts_per_model,
            verifying_attempts=arguments.verifying_attempts,
        )
    else:
        runs_of_objects = ParallelRoboCasaPolicyIteration(
            **common,
            workers_per_iteration=arguments.workers_per_iteration,
            results=arguments.results,
            iteration_arguments=iteration_arguments,
        )
    for name, last_line in runs_of_objects.run().items():
        print(f"{name}: {last_line}")


if __name__ == "__main__":
    main()

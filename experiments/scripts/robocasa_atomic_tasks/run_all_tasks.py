"""
Run every one of RoboCasa's atomic tasks, each in a process of its own running
``run_task.py``, and report which the robot accomplished.

Run with::

    python experiments/scripts/robocasa_atomic_tasks/run_all_tasks.py --workers 8 --results ~/atomic_tasks
    python experiments/scripts/robocasa_atomic_tasks/run_all_tasks.py --tasks OpenDrawer TurnOnStove
    python experiments/scripts/robocasa_atomic_tasks/run_all_tasks.py --videos --physics
    python experiments/scripts/robocasa_atomic_tasks/run_all_tasks.py --physics --robot panda_omron

Every task writes its log, and with ``--videos`` a video seen from the robot's
camera, into the results directory; the outcomes are collected there too.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.command_line import (
    ResultFile,
    TaskExitStatus,
    TaskRunOption,
)
from experiments.robocasa_atomic_tasks.robots import RobotModel
from krrood.adapters.json_serializer import DataclassJSONSerializer

# %% one task's run


@dataclass
class TaskOutcome:
    """
    How the run of one task ended.
    """

    task: str
    """
    The name of the task.
    """

    succeeded: bool
    """
    Whether RoboCasa's own check found the task accomplished.
    """

    seconds: float
    """
    How long the run took.
    """

    exit_status: int
    """
    The exit status of the task's process, see
    :class:`~experiments.robocasa_atomic_tasks.command_line.TaskExitStatus`; negative if
    a signal ended it, such as the alarm of its time limit.
    """


@dataclass
class TaskRun:
    """
    How every task is run.
    """

    results: Path
    """
    The directory the tasks' logs, videos and outcomes are written to.
    """

    seed: int
    """
    The seed RoboCasa samples the tasks' scenes with.
    """

    time_limit: int
    """
    How many seconds a task may run.
    """

    video: bool
    """
    Whether every task records a video of itself.
    """

    physics: bool
    """
    Whether the tasks are simulated physically in MuJoCo.
    """

    robot: RobotModel
    """
    The robot that performs the tasks.
    """

    def run(self, task: str) -> TaskOutcome:
        """
        :param task: The name of a task.
        :return: How its run, in a process of its own, ended.
        """
        start = time.time()
        command = [
            sys.executable,
            str(Path(__file__).with_name("run_task.py")),
            task,
            TaskRunOption.SEED,
            str(self.seed),
            TaskRunOption.TIME_LIMIT,
            str(self.time_limit),
            TaskRunOption.ROBOT,
            self.robot,
        ]
        if self.video:
            command += [
                TaskRunOption.VIDEO,
                str(self.results / ResultFile.VIDEO.for_task(task)),
            ]
        if self.physics:
            command.append(TaskRunOption.PHYSICS)
        with (self.results / ResultFile.LOG.for_task(task)).open("w") as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        return TaskOutcome(
            task=task,
            succeeded=process.returncode == TaskExitStatus.ACCOMPLISHED,
            seconds=round(time.time() - start),
            exit_status=process.returncode,
        )


# %% the command line

tasks = TaskModule.tasks_by_name
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--tasks", nargs="*", choices=sorted(tasks), default=None)
parser.add_argument("--results", type=Path, default=Path("atomic_task_results"))
parser.add_argument("--workers", type=int, default=4)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--time-limit", type=int, default=1800)
parser.add_argument("--videos", action="store_true")
parser.add_argument("--physics", action="store_true")
parser.add_argument(
    "--robot", type=RobotModel, choices=list(RobotModel), default=RobotModel.PR2
)
arguments = parser.parse_args()

# %% running every task

arguments.results.mkdir(parents=True, exist_ok=True)
task_run = TaskRun(
    results=arguments.results,
    seed=arguments.seed,
    time_limit=arguments.time_limit,
    video=arguments.videos,
    physics=arguments.physics,
    robot=arguments.robot,
)
with ThreadPoolExecutor(arguments.workers) as pool:
    outcomes = list(pool.map(task_run.run, arguments.tasks or sorted(tasks)))
(arguments.results / ResultFile.OUTCOMES).write_text(
    json.dumps(
        [DataclassJSONSerializer.to_json(instance=outcome) for outcome in outcomes], indent=2
    )
)
for outcome in outcomes:
    print(
        f"{outcome.task:32} {'succeeded' if outcome.succeeded else 'failed':10}"
        f" {outcome.seconds:6.0f} s"
    )
print(
    f"{sum(outcome.succeeded for outcome in outcomes)} of {len(outcomes)} accomplished"
)

"""
Run every one of RoboCasa's atomic tasks, each in a process of its own, and report which
the PR2 accomplished.

Run with::

    python -m experiments.robocasa_atomic_tasks --workers 8 --results ~/atomic_tasks
    python -m experiments.robocasa_atomic_tasks --tasks OpenDrawer TurnOnStove
    python -m experiments.robocasa_atomic_tasks --videos --results ~/atomic_tasks
    python -m experiments.robocasa_atomic_tasks --physics --results ~/atomic_tasks

Every task writes its log to ``<results>/<task>.log``, and with ``--videos`` a video
seen from the PR2's head camera to ``<results>/<task>.mp4``; the outcomes are collected
in ``<results>/outcomes.json``.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass
from pathlib import Path

from typing_extensions import Dict, List

from experiments.robocasa_atomic_tasks import (
    blender,
    coffee,
    doors,
    drawer,
    electric_kettle,
    microwave,
    navigate,
    oven,
    pick_place,
    sink,
    stand_mixer,
    stove,
    toaster,
    toaster_oven,
)

MODULES = [
    blender,
    coffee,
    doors,
    drawer,
    electric_kettle,
    microwave,
    navigate,
    oven,
    pick_place,
    sink,
    stand_mixer,
    stove,
    toaster,
    toaster_oven,
]
"""
The modules of the tasks, one per file RoboCasa defines its atomic tasks in.
"""


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

    return_code: int
    """
    The exit status of the task's process; 0 unless it failed with an error.
    """


def tasks_by_name() -> Dict[str, str]:
    """
    :return: The module every task is defined in, by the name of the task.
    """
    return {
        task.__name__: module.__name__ for module in MODULES for task in module.TASKS
    }


def run_task(
    task: str,
    module: str,
    results: Path,
    seed: int,
    timeout: float,
    video: bool,
    physics: bool,
) -> TaskOutcome:
    """
    Run one task in a process of its own.

    :param task: The name of the task.
    :param module: The module it is defined in.
    :param results: The directory the task's log is written to.
    :param seed: The seed RoboCasa samples the task's scene with.
    :param timeout: How long the task may run, in seconds.
    :param video: Whether the task records a video of itself, next to its log.
    :param physics: Whether the task is simulated physically in MuJoCo.
    :return: How the run ended.
    """
    start = time.time()
    log = results / f"{task}.log"
    command = [sys.executable, "-m", module, task, "--seed", str(seed)]
    if video:
        command += ["--video", str(results / f"{task}.mp4")]
    if physics:
        command.append("--physics")
    with log.open("w") as output:
        try:
            process = subprocess.run(
                command,
                stdout=output,
                stderr=subprocess.STDOUT,
                timeout=timeout,
            )
            return_code = process.returncode
        except subprocess.TimeoutExpired:
            return_code = -1
    succeeded = f"{task} succeeded after" in log.read_text(errors="replace")
    return TaskOutcome(task, succeeded, round(time.time() - start), return_code)


def main() -> None:
    """
    Run the tasks the command line selects and report their outcomes.
    """
    names = tasks_by_name()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tasks", nargs="*", choices=sorted(names), default=None)
    parser.add_argument("--results", type=Path, default=Path("atomic_task_results"))
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--timeout", type=float, default=1800.0, help="Seconds a task may run."
    )
    parser.add_argument(
        "--videos",
        action="store_true",
        help="Have every task record a video seen from the PR2's head camera.",
    )
    parser.add_argument(
        "--physics",
        action="store_true",
        help="Simulate the PR2 and the kitchen physically in MuJoCo.",
    )
    arguments = parser.parse_args()
    arguments.results.mkdir(parents=True, exist_ok=True)
    selected = arguments.tasks or sorted(names)
    with ThreadPoolExecutor(arguments.workers) as executor:
        outcomes: List[TaskOutcome] = list(
            executor.map(
                lambda task: run_task(
                    task,
                    names[task],
                    arguments.results,
                    arguments.seed,
                    arguments.timeout,
                    arguments.videos,
                    arguments.physics,
                ),
                selected,
            )
        )
    (arguments.results / "outcomes.json").write_text(
        json.dumps([asdict(outcome) for outcome in outcomes], indent=2)
    )
    for outcome in outcomes:
        print(
            f"{outcome.task:32} {'succeeded' if outcome.succeeded else 'failed':10}"
            f" {outcome.seconds:6.0f} s"
        )
    accomplished = sum(outcome.succeeded for outcome in outcomes)
    print(f"{accomplished} of {len(outcomes)} tasks accomplished")


if __name__ == "__main__":
    main()

"""
Have a robot perform one of RoboCasa's atomic tasks and report whether RoboCasa finds it
accomplished, exiting with the matching
:class:`~experiments.robocasa_atomic_tasks.command_line.TaskExitStatus`.

Run with::

    python experiments/scripts/robocasa_atomic_tasks/run_task.py OpenDrawer
    python experiments/scripts/robocasa_atomic_tasks/run_task.py TurnOnStove --physics --video stove.mp4
    python experiments/scripts/robocasa_atomic_tasks/run_task.py AdjustWaterTemperature --robot panda_omron
    python experiments/scripts/robocasa_atomic_tasks/run_task.py AdjustWaterTemperature --robot panda_omron --physics --database sqlite:////root/work/task_runs.sqlite
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time
from pathlib import Path

from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.command_line import TaskExitStatus, TaskRunOption
from experiments.robocasa_atomic_tasks.robots import RobotModel
from experiments.robocasa_atomic_tasks.task_run_database import TaskRunDatabase
from semantic_digital_twin.adapters.robocasa_dataset.mujoco_compat import (
    robocasa_version_assertions_relaxed,
)

with robocasa_version_assertions_relaxed():
    from robocasa.models.scenes.scene_registry import LayoutType, StyleType

logger = logging.getLogger(__name__)

# %% the command line

tasks = TaskModule.tasks_by_name
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("task", choices=sorted(tasks))
parser.add_argument(
    TaskRunOption.LAYOUT, choices=[layout.name for layout in LayoutType]
)
parser.add_argument(TaskRunOption.STYLE, choices=[style.name for style in StyleType])
parser.add_argument(TaskRunOption.SEED, type=int, default=0)
parser.add_argument(TaskRunOption.PHYSICS, action="store_true")
parser.add_argument(TaskRunOption.VIDEO, type=Path)
parser.add_argument(
    TaskRunOption.COLLISION_AVOIDANCE,
    action=argparse.BooleanOptionalAction,
    default=True,
)
parser.add_argument(
    TaskRunOption.ROBOT,
    type=RobotModel,
    choices=list(RobotModel),
    default=RobotModel.PR2,
)
parser.add_argument(TaskRunOption.TIME_LIMIT, type=int, default=0)
parser.add_argument(TaskRunOption.DATABASE)
arguments = parser.parse_args()

# %% running the task

logging.basicConfig(level=logging.INFO, force=True)
# the default action of the alarm signal ends the process once the time limit is up
signal.alarm(arguments.time_limit)
settings = dict(
    seed=arguments.seed,
    collision_avoidance=arguments.collision_avoidance,
    physics=arguments.physics,
    video=arguments.video,
    robot=arguments.robot,
    records_the_run=arguments.database is not None,
)
if arguments.layout:
    settings["layout"] = LayoutType[arguments.layout]
if arguments.style:
    settings["style"] = StyleType[arguments.style]
start = time.time()
task = tasks[arguments.task](**settings)
succeeded = task.run()
logger.info(
    "%s %s after %.0f s",
    arguments.task,
    "succeeded" if succeeded else "failed",
    time.time() - start,
)
if task.recorded_run is not None:
    TaskRunDatabase.connect(arguments.database).add(task.recorded_run)
    logger.info(
        "Stored the run, %d moments, in %s",
        len(task.recorded_run.moments),
        arguments.database,
    )
sys.exit(TaskExitStatus.ACCOMPLISHED if succeeded else TaskExitStatus.NOT_ACCOMPLISHED)

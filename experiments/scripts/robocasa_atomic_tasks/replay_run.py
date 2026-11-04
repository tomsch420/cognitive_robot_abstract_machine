"""
Replay the run of one of RoboCasa's atomic tasks stored last, in an episode composed
anew in the kitchen it was performed in, as fast as it passed in the simulation.

Run under cramera to watch it::

    cramera-live experiments/scripts/robocasa_atomic_tasks/replay_run.py AdjustWaterTemperature -- --database sqlite:////root/work/task_runs.sqlite
"""

from __future__ import annotations

import argparse
import logging

from experiments.robocasa_atomic_tasks.catalogue import TaskModule
from experiments.robocasa_atomic_tasks.command_line import TaskReplayOption
from experiments.robocasa_atomic_tasks.stored_runs import TaskRunReplay
from experiments.robocasa_atomic_tasks.task_run_database import TaskRunDatabase

# %% the command line

tasks = TaskModule.tasks_by_name
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("task", choices=sorted(tasks))
parser.add_argument(TaskReplayOption.DATABASE, required=True)
parser.add_argument(TaskReplayOption.SPEED, type=float, default=1.0)
arguments = parser.parse_args()

# %% replaying the run

logging.basicConfig(level=logging.INFO, force=True)
run = TaskRunDatabase.connect(arguments.database).latest_run_of(arguments.task)
episode = tasks[run.task](
    layout=run.layout, style=run.style, seed=run.seed, robot=run.robot
).compose()
TaskRunReplay(world=episode.world, run=run).play(arguments.speed)

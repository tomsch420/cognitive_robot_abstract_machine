"""
Replay a stored attempt of the kitchen grasp trials, with the plans the robot performed,
in its kitchen cut down anew, as fast as it passed in the simulation.

Run under cramera to watch it::

    cramera-live experiments/scripts/grasp_learning/replay_kitchen_grasp_attempt.py -- --database sqlite:////root/work/attempts.sqlite 1
"""

from __future__ import annotations

import argparse
import logging

from experiments.grasp_learning.database import GraspDatabase
from experiments.grasp_learning.kitchen_grasp_trials import replay_run

# %% the command line

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--database", required=True, help="the database URI")
parser.add_argument("attempt_id", type=int, help="the database id of the attempt")
parser.add_argument("--speed", type=float, default=1.0)
arguments = parser.parse_args()

# %% replaying the attempt

logging.basicConfig(level=logging.INFO, force=True)
replay_run(
    run=GraspDatabase.connect(arguments.database)
    .recorded_attempt(arguments.attempt_id)
    .run,
    speed=arguments.speed,
)

#!/bin/bash
# Tries grasps on the object of a pick and place task in its kitchen cut down to the
# fixture the object lies in, with many workers at once against one shared database:
# each worker runs experiments.grasp_learning.kitchen_grasp_trials with grasps drawn
# under a seed of its own.
#
# Usage: run_kitchen_grasp_trials.sh WORKERS ATTEMPTS_PER_WORKER [FIRST_SEED]
#   WORKERS              how many workers run at once
#   ATTEMPTS_PER_WORKER  how many grasps each worker draws and tries
#   FIRST_SEED           the draw seed of the first worker; the others follow it
#                        (default 0)
#
# Configured by environment variables:
#   GRASP_LEARNING_DATABASE_URI
#                        the database every worker stores its attempts in (required)
#   LOG_DIRECTORY        where each worker writes worker_<seed>.log (required)
#   THREADS_PER_WORKER   how many threads the numerical libraries of a worker may start
#                        (default 4)
#   TRIAL_ARGUMENTS      further arguments of the trials, such as --task
#
# The workers run in the background; each log ends in a line saying how many of its
# attempts lifted the object.
set -euo pipefail

WORKERS=${1:?give the number of workers}
ATTEMPTS=${2:?give the attempts per worker}
FIRST_SEED=${3:-0}
: "${GRASP_LEARNING_DATABASE_URI:?set GRASP_LEARNING_DATABASE_URI to the database}"
: "${LOG_DIRECTORY:?set LOG_DIRECTORY to where the logs go}"
THREADS=${THREADS_PER_WORKER:-4}

mkdir -p "$LOG_DIRECTORY"
export OMP_NUM_THREADS=$THREADS OPENBLAS_NUM_THREADS=$THREADS MKL_NUM_THREADS=$THREADS
export NUMBA_NUM_THREADS=$THREADS MALLOC_ARENA_MAX=4
for ((seed = FIRST_SEED; seed < FIRST_SEED + WORKERS; seed++)); do
  # shellcheck disable=SC2086
  setsid nohup python3 -m experiments.grasp_learning.kitchen_grasp_trials \
    --database "$GRASP_LEARNING_DATABASE_URI" \
    --attempts "$ATTEMPTS" --draw-seed "$seed" ${TRIAL_ARGUMENTS:-} \
    > "$LOG_DIRECTORY/worker_$seed.log" 2>&1 < /dev/null &
done
echo "started $WORKERS workers, draw seeds $FIRST_SEED to $((FIRST_SEED + WORKERS - 1))"

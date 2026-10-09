#!/bin/bash
# Learns grasp models for several tasks in parallel: one Docker container per task, each
# running the grasp learning pipeline once against a shared Postgres database.
#
# Usage: run_tasks.sh [ATTEMPTS] [VERIFYING_ATTEMPTS] [ROBOT:OBJECT ...]
#   ATTEMPTS             grasps drawn from the stated regions per task (default 20)
#   VERIFYING_ATTEMPTS   grasps drawn from the learned model per task (default 5)
#   ROBOT:OBJECT         the tasks (default: the nine tasks below)
#
# Configured by environment variables:
#   CRAM_REPOSITORY      the checkout, mounted at /opt/cram (required)
#   CRAM_HOME            the directory mounted as /root (required)
#   GRASP_LEARNING_DATABASE_URI
#                        the database, as the containers reach it (required)
#   DOCKER_NETWORK       the network the database is reachable on (required)
#   CRAM_IMAGE           the image to run (default cram:latest)
#   WORKER_PREFIX        prefix of the worker container names
#                        (default grasp-learning-worker)
#   LOG_DIRECTORY        where the logs go, relative to CRAM_HOME
#                        (default grasp_learning_logs)
#
# Each worker writes its log to $CRAM_HOME/$LOG_DIRECTORY/<robot>_<object>.log,
# ending in a line saying what its model was learned from and how often its grasps
# lifted the object. Print the latest model of every task with
#   python -m experiments.grasp_learning.report
set -euo pipefail

ATTEMPTS=${1:-20}
VERIFYING_ATTEMPTS=${2:-5}
shift $(($# < 2 ? $# : 2))
TASKS=("$@")
if [ ${#TASKS[@]} -eq 0 ]; then
  TASKS=(pr2:bowl pr2:cup pr2:milk pr2:spoon pr2:ycb_mug
         tracy:bowl tracy:cup tracy:milk tracy:ycb_mug)
fi
: "${CRAM_REPOSITORY:?set CRAM_REPOSITORY to the checkout}"
: "${CRAM_HOME:?set CRAM_HOME to the directory mounted as /root}"
: "${GRASP_LEARNING_DATABASE_URI:?set GRASP_LEARNING_DATABASE_URI}"
: "${DOCKER_NETWORK:?set DOCKER_NETWORK to the network the database is on}"
CRAM_IMAGE=${CRAM_IMAGE:-cram:latest}
WORKER_PREFIX=${WORKER_PREFIX:-grasp-learning-worker}
LOG_DIRECTORY=${LOG_DIRECTORY:-grasp_learning_logs}

# Sources the environment of the image and installs what it lacks, then runs $1.
in_container() {
  echo "source /opt/ros/cram-env/bin/activate \
    && source /opt/ros/overlay_ws/install/setup.bash \
    && UV_CACHE_DIR=/root/work/.uv_cache uv pip install -q shapely \
    && cd /opt/cram && $1"
}

mkdir -p "$CRAM_HOME/$LOG_DIRECTORY"
common=(--network "$DOCKER_NETWORK" -v "$CRAM_REPOSITORY:/opt/cram" -v "$CRAM_HOME:/root"
        -e GRASP_LEARNING_DATABASE_URI="$GRASP_LEARNING_DATABASE_URI")

# The workers would race to create the tables of an empty database.
docker run --rm "${common[@]}" "$CRAM_IMAGE" bash -lc \
  "$(in_container "python -m experiments.grasp_learning.database")"

for task in "${TASKS[@]}"; do
  robot=${task%%:*}
  object=${task##*:}
  docker run -d --name "$WORKER_PREFIX-$robot-$object" "${common[@]}" \
    --gpus all --shm-size 2g -e MUJOCO_GL=egl -e CI=true -e OPENBLAS_NUM_THREADS=16 \
    "$CRAM_IMAGE" bash -lc "$(in_container "python -m experiments.grasp_learning.pipeline \
      --robot $robot --object $object --attempts $ATTEMPTS \
      --verifying-attempts $VERIFYING_ATTEMPTS \
      > /root/$LOG_DIRECTORY/${robot}_${object}.log 2>&1")"
done
docker ps --filter "name=$WORKER_PREFIX" --format '{{.Names}} {{.Status}}'

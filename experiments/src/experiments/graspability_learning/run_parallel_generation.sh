#!/bin/bash
# Orchestrates parallel multi-worker dataset generation: N workers per kind, each
# writing its own shard DB, then merges all shards into the final combined DB.
set -e
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /opt/ros/overlay_ws/install/setup.bash 2>/dev/null
cd /opt/cram

WORKERS_PER_KIND=10
AMOUNT_PER_WORKER=170
SHARD_DIR=/tmp/grasp_shards
FINAL_DB=experiments/src/experiments/graspability_learning/resources/grasp_dataset_multi.sqlite
LOG=/tmp/generate_dataset_multi.log

rm -rf "$SHARD_DIR"
mkdir -p "$SHARD_DIR"

echo "Launching $((WORKERS_PER_KIND * 3)) workers ($WORKERS_PER_KIND per kind x $AMOUNT_PER_WORKER trials each = $((WORKERS_PER_KIND * AMOUNT_PER_WORKER * 3)) total)" >> "$LOG"

for kind in CUBE MILK CUP; do
  for w in $(seq 0 $((WORKERS_PER_KIND - 1))); do
    worker_id="${kind}_${w}"
    python3 -u experiments/src/experiments/graspability_learning/generate_dataset_worker.py \
      --kind "$kind" --amount "$AMOUNT_PER_WORKER" \
      --db-path "$SHARD_DIR/shard_${worker_id}.sqlite" \
      --worker-id "$worker_id" --log-every 50 \
      > "$SHARD_DIR/worker_${worker_id}.log" 2>&1 &
  done
done

wait
echo "All workers finished, merging shards..." >> "$LOG"

python3 -u experiments/src/experiments/graspability_learning/merge_shards.py \
  --shard-glob "$SHARD_DIR/shard_*.sqlite" --out-db "$FINAL_DB" >> "$LOG" 2>&1

n_errors=$(grep -c "TRIAL RAISED" "$SHARD_DIR"/worker_*.log | awk -F: '{s+=$2} END {print s+0}')
echo "DONE: parallel generation complete, db=$FINAL_DB, total_errors_across_workers=$n_errors" >> "$LOG"

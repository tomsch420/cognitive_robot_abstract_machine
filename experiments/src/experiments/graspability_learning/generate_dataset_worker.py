"""
Single-kind, single-shard worker for parallel dataset generation. Each worker builds
its own World (MuJoCo/Giskard state isn't shareable across processes anyway) and
writes to its own SQLite file -- SQLite doesn't handle concurrent writers well, so
sharding by file (merged afterward by merge_shards.py) avoids write contention
entirely rather than trying to serialize many processes onto one DB file.
"""
import os, sys, time, argparse

# Must happen before any giskardpy/rclpy import touches ROS: many parallel workers
# each call rospy.init_node(...) (see giskard_reach.py), and left on the same default
# ROS_DOMAIN_ID they contend over DDS shared-memory transport segments -- observed
# directly as `RTPS_TRANSPORT_SHM Error: Failed to create segment ... library_error`
# under a 30-worker run, which correlated with milk/cup (whose real-physics success
# needs a very precise reach, unlike the cube's much wider margin) dropping to exactly
# 0/1700 successes even though the same candidates succeed fine run alone. Giving each
# worker process its own ROS domain removes the contention at the source rather than
# hoping the degraded transport doesn't matter.
os.environ["ROS_DOMAIN_ID"] = str((os.getpid() % 200) + 1)
sys.path.insert(0, "experiments/src")

from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine
from krrood.ormatic.data_access_objects.helper import to_dao

import experiments.orm.ormatic_interface as orm_interface
from experiments.graspability_learning.domain_model import GraspableKind
from experiments.graspability_learning.stage1_fixed_sample import build_world, run_fixed_candidate

KIND_MAP = {"CUBE": GraspableKind.CUBE, "MILK": GraspableKind.MILK, "CUP": GraspableKind.CUP}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kind", required=True, choices=list(KIND_MAP))
    parser.add_argument("--amount", type=int, required=True)
    parser.add_argument("--db-path", required=True)
    parser.add_argument("--log-every", type=int, default=50)
    parser.add_argument("--worker-id", default="0")
    args = parser.parse_args()

    kind = KIND_MAP[args.kind]
    engine = create_engine(f"sqlite:///{args.db_path}")
    orm_interface.Base.metadata.create_all(engine)
    session = sessionmaker(engine)()

    grasp_world = build_world(kind)
    graspable = grasp_world.graspable

    n_success = 0
    n_error = 0
    start = time.time()
    for i, candidate in enumerate(graspable.grasp_candidates(end_effector=None, amount=args.amount)):
        try:
            result = run_fixed_candidate(candidate, grasp_world)
        except Exception as e:
            n_error += 1
            print(f"[w{args.worker_id} {kind.value} {i}] TRIAL RAISED: {type(e).__name__}: {e}", flush=True)
            continue

        session.add(to_dao(result))
        session.commit()
        session.expunge_all()

        n_success += int(result.success)
        if i % args.log_every == 0 or i == args.amount - 1:
            elapsed = time.time() - start
            print(
                f"[w{args.worker_id} {kind.value} {i + 1}/{args.amount}] "
                f"success_rate={n_success / (i + 1):.2%} errors={n_error} elapsed={elapsed:.0f}s",
                flush=True,
            )

    print(
        f"[w{args.worker_id} {kind.value}] WORKER DONE: {n_success}/{args.amount} successful, "
        f"{n_error} errors, db={args.db_path}",
        flush=True,
    )


if __name__ == "__main__":
    main()

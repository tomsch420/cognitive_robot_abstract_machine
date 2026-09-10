"""
Generate a dataset of grasp trials across all three GraspableKind geometries (Cube,
milk carton, cup), persisting each GraspTrialResult (and its nested GraspCandidate/
graspable object) to a SQLite database via ORMatic, one commit per item (never
accumulated -- see helper.to_dao/session.add/commit/expunge_all below).

Writes to a NEW database file by default (grasp_dataset_multi.sqlite) rather than the
original single-cube grasp_dataset.sqlite: the schema gained a new `graspable_kind`
column (GraspCandidate), and SQLAlchemy's create_all only creates missing tables, it
does not migrate existing ones -- reusing the old file would leave that column absent
from the actual table. The original 5000-row cube-only dataset is left untouched.
"""
import sys, time, argparse
sys.path.insert(0, "experiments/src")

from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine
from krrood.ormatic.data_access_objects.helper import to_dao

import experiments.orm.ormatic_interface as orm_interface
from experiments.graspability_learning.domain_model import GraspableKind
from experiments.graspability_learning.stage1_fixed_sample import build_world, run_fixed_candidate

DEFAULT_DB_PATH = "experiments/src/experiments/graspability_learning/resources/grasp_dataset_multi.sqlite"
KINDS = [GraspableKind.CUBE, GraspableKind.MILK, GraspableKind.CUP]


def run_kind(kind, amount, session, log_every, offset, total):
    grasp_world = build_world(kind)
    graspable = grasp_world.graspable

    n_success = 0
    n_error = 0
    start = time.time()
    for i, candidate in enumerate(graspable.grasp_candidates(end_effector=None, amount=amount)):
        try:
            result = run_fixed_candidate(candidate, grasp_world)
        except Exception as e:
            n_error += 1
            print(f"[{kind.value} {i}] TRIAL RAISED: {type(e).__name__}: {e}")
            continue

        session.add(to_dao(result))
        session.commit()
        session.expunge_all()

        n_success += int(result.success)
        if i % log_every == 0 or i == amount - 1:
            elapsed = time.time() - start
            rate = (i + 1) / elapsed
            eta = (amount - i - 1) / rate if rate > 0 else float("inf")
            print(
                f"[{kind.value} {offset + i + 1}/{total}] success_rate={n_success / (i + 1):.2%} "
                f"errors={n_error} elapsed={elapsed:.0f}s eta={eta:.0f}s",
                flush=True,
            )
    print(f"{kind.value} DONE: {n_success}/{amount} successful, {n_error} errors", flush=True)
    return n_success, n_error


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--amount-per-kind", type=int, default=1700)
    parser.add_argument("--db-path", default=DEFAULT_DB_PATH)
    parser.add_argument("--log-every", type=int, default=50)
    args = parser.parse_args()

    engine = create_engine(f"sqlite:///{args.db_path}")
    orm_interface.Base.metadata.create_all(engine)
    session = sessionmaker(engine)()

    total = args.amount_per_kind * len(KINDS)
    grand_success = 0
    grand_error = 0
    offset = 0
    for kind in KINDS:
        n_success, n_error = run_kind(kind, args.amount_per_kind, session, args.log_every, offset, total)
        grand_success += n_success
        grand_error += n_error
        offset += args.amount_per_kind

    print(f"DONE: {grand_success}/{total} successful, {grand_error} errors, db={args.db_path}")


if __name__ == "__main__":
    main()

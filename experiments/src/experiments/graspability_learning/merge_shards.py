"""
Merge many per-worker shard SQLite databases (see generate_dataset_worker.py) into
one combined database, by round-tripping each GraspTrialResultDAO through its domain
object and re-persisting it (matching the same per-item commit+expunge pattern used
everywhere else in this pipeline).
"""
import sys, glob, argparse
sys.path.insert(0, "experiments/src")

from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine
from krrood.ormatic.data_access_objects.helper import to_dao

import experiments.orm.ormatic_interface as orm_interface
from experiments.orm.ormatic_interface import GraspTrialResultDAO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--shard-glob", required=True)
    parser.add_argument("--out-db", required=True)
    args = parser.parse_args()

    out_engine = create_engine(f"sqlite:///{args.out_db}")
    orm_interface.Base.metadata.create_all(out_engine)
    out_session = sessionmaker(out_engine)()

    total = 0
    for shard_path in sorted(glob.glob(args.shard_glob)):
        engine = create_engine(f"sqlite:///{shard_path}")
        session = sessionmaker(engine)()
        instances = list(session.scalars(select(GraspTrialResultDAO)).all())
        for dao in instances:
            domain_obj = dao.from_dao()
            out_session.add(to_dao(domain_obj))
            out_session.commit()
            out_session.expunge_all()
            total += 1
        session.close()
        print(f"merged {len(instances)} from {shard_path}, running total={total}", flush=True)

    print(f"MERGE DONE: {total} total trials in {args.out_db}", flush=True)


if __name__ == "__main__":
    main()

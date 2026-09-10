"""
Validate the fitted RSPN by comparing sampled statistics against real dataset
statistics -- proof the circuit learned real structure, not just fit without error.
Broken down per GraspableKind, since that's the whole point of widening the dataset
beyond the cube.

Enum columns (graspable_kind) go through krrood's FeatureExtractor.preprocess_dataframe
as `hash(enum_member)`. Python's default Enum hash is identity-based (`object.__hash__`,
effectively derived from the object's memory address), which is NOT stable across
separate process runs -- and empirically, comparing a freshly-computed
`hash(GraspableKind.MILK)` against the circuit's sampled values (even within this same
process) did not reliably match for MILK/CUP even though it happened to match for CUBE,
suggesting the DB round-trip doesn't always hand back the exact canonical singleton
object for every enum column. Rather than depend on hash() being reproducible at all,
this builds the hash->kind-name mapping empirically from the *training instances
themselves* (each DAO's `.candidate.graspable_kind.name` is a plain, always-stable
string) in this same process, then uses that mapping to interpret circuit.sample()'s
output -- sidestepping the instability entirely instead of fighting it.
"""
import sys
sys.path.insert(0, "experiments/src")

import numpy as np
from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine

from experiments.orm.ormatic_interface import GraspTrialResultDAO
from experiments.graspability_learning.domain_model import GraspTrialResult, GraspableKind
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)

DB_PATH = "experiments/src/experiments/graspability_learning/resources/grasp_dataset_multi.sqlite"
KINDS = [GraspableKind.CUBE, GraspableKind.MILK, GraspableKind.CUP]


def main():
    engine = create_engine(f"sqlite:///{DB_PATH}")
    session = sessionmaker(engine)()
    instances = list(session.scalars(select(GraspTrialResultDAO)).all())

    # Empirical hash->name mapping, built from the exact same objects preprocess_dataframe
    # will hash during fit() below -- guaranteed self-consistent within this process,
    # regardless of whether Enum hash matches anything computed anywhere else.
    hash_to_name = {}
    for inst in instances:
        kind_attr = inst.candidate.graspable_kind
        hash_to_name[hash(kind_attr)] = kind_attr.name
    print("Empirical hash->kind mapping observed in training data:", hash_to_name)

    model = RelationalProbabilisticCircuit(GraspTrialResult)
    model.fit(instances)
    circuit = model.class_probabilistic_circuit

    names = [v.name for v in circuit.variables]
    aperture_idx = names.index("GraspTrialResult.candidate.aperture")
    success_idx = names.index("GraspTrialResult.success")
    kind_idx = names.index("GraspTrialResult.candidate.graspable_kind")

    samples = np.asarray(circuit.sample(9000))
    unique_sampled = np.unique(samples[:, kind_idx])
    print("Unique sampled kind-hash values:", unique_sampled)

    import sqlite3
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute(
        """
        SELECT c.aperture, t.success, t.max_translation_slip, c.graspable_kind
        FROM GraspTrialResultDAO t JOIN GraspCandidateDAO c ON t.candidate_id = c.database_id
        """
    )
    real_rows = cur.fetchall()

    print(f"\n{'kind':6} {'real P(success)':>16} {'sampled P(success)':>19} "
          f"{'real n':>8} {'sampled n':>10}")
    for kind in KINDS:
        real_kind_rows = [r for r in real_rows if r[3].endswith(kind.name)]
        real_success = [r for r in real_kind_rows if r[1]]
        real_p_success = len(real_success) / len(real_kind_rows) if real_kind_rows else float("nan")

        # Find the sampled hash value(s) whose empirical mapping matches this kind's name.
        matching_hashes = [h for h, name in hash_to_name.items() if name == kind.name]
        sampled_mask = np.isin(samples[:, kind_idx], matching_hashes)
        sampled_kind = samples[sampled_mask]
        sampled_p_success = (
            (sampled_kind[:, success_idx] > 0.5).mean() if len(sampled_kind) else float("nan")
        )

        print(f"{kind.value:6} {real_p_success:16.2%} {sampled_p_success:19.2%} "
              f"{len(real_kind_rows):8d} {len(sampled_kind):10d}")


if __name__ == "__main__":
    main()

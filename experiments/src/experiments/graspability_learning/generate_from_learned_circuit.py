"""
Compare the random generator (Cube.grasp_candidates' current ProbabilisticBackend,
FullyFactorizedRegistry) against the same EQL statement evaluated through
ProbabilisticBackend(model_registry=RelationalCircuitRegistry(...)) using the fitted
RelationalProbabilisticCircuit, conditioned on success=True -- checking whether the
resulting candidates' real physics success rate is actually higher.
"""
import sys, itertools
sys.path.insert(0, "experiments/src")

from sqlalchemy import select
from sqlalchemy.orm import sessionmaker

from krrood.ormatic.utils import create_engine
from krrood.entity_query_language.factories import a
from krrood.entity_query_language.backends import ProbabilisticBackend
from krrood.parametrization.model_registries import RelationalCircuitRegistry

from experiments.orm.ormatic_interface import GraspTrialResultDAO
from experiments.graspability_learning.domain_model import GraspCandidate, GraspTrialResult
from experiments.graspability_learning.stage1_fixed_sample import build_world, run_fixed_candidate
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)

DB_PATH = "experiments/src/experiments/graspability_learning/resources/grasp_dataset.sqlite"
N_TRIALS = 40


def main():
    engine = create_engine(f"sqlite:///{DB_PATH}")
    session = sessionmaker(engine)()
    instances = list(session.scalars(select(GraspTrialResultDAO)).all())

    fitted = RelationalProbabilisticCircuit(GraspTrialResult)
    fitted.fit(instances)
    print("Circuit fitted.")

    world, cube = build_world()

    query = a(GraspTrialResult)(
        candidate=a(GraspCandidate)(graspable=cube, aperture=..., closing_effort=...),
        success=True,
    )
    query = query.where(
        query.variable.candidate.aperture >= 0.0,
        query.variable.candidate.aperture <= 0.04,
        query.variable.candidate.closing_effort >= 5.0,
        query.variable.candidate.closing_effort <= 60.0,
    )
    print("QUERY:", query)

    registry = RelationalCircuitRegistry(relational_probabilistic_circuit=fitted)
    backend = ProbabilisticBackend(model_registry=registry, number_of_samples=N_TRIALS)

    generated = list(itertools.islice(backend.evaluate(query), N_TRIALS))
    print(f"Got {len(generated)} samples from the learned-circuit generator")
    for g in generated[:5]:
        print(" sample:", g)

    n_success = 0
    for i, sample in enumerate(generated):
        candidate = sample.candidate if hasattr(sample, "candidate") else sample
        candidate.graspable = cube
        candidate.end_effector = None
        candidate.approach_pose = cube._default_approach_pose()
        result = run_fixed_candidate(candidate, world, cube)
        n_success += int(result.success)
        print(f"[{i}] aperture={candidate.aperture:.4f} closing_effort={candidate.closing_effort:.4f} "
              f"-> success={result.success} slip={result.max_translation_slip:.4f}")

    print(f"\nLEARNED-CIRCUIT GENERATOR: {n_success}/{len(generated)} = {n_success/len(generated):.2%} success rate")
    print("RANDOM GENERATOR BASELINE (from the 5000-sample dataset): 33.52%")


if __name__ == "__main__":
    main()

"""
Control: identical query and backend, but WITHOUT success=True, to check whether that
literal is actually the thing responsible for the aperture bias / high success rate, or
whether something else is going on.
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

    # SAME query, conditioned on success=False instead of True -- a fully well-formed
    # control that doesn't need further schema changes. If success=True was really
    # doing the conditioning (not some incidental unconditioned bias), this should push
    # apertures toward the LOW end and the real success rate DOWN, not up.
    query = a(GraspTrialResult)(
        candidate=a(GraspCandidate)(graspable=cube, aperture=..., closing_effort=...),
        success=False,
    )
    query = query.where(
        query.variable.candidate.aperture >= 0.0,
        query.variable.candidate.aperture <= 0.04,
        query.variable.candidate.closing_effort >= 5.0,
        query.variable.candidate.closing_effort <= 60.0,
    )
    print("QUERY (conditioned on success=False):", query)

    registry = RelationalCircuitRegistry(relational_probabilistic_circuit=fitted)
    backend = ProbabilisticBackend(model_registry=registry, number_of_samples=N_TRIALS)

    generated = list(itertools.islice(backend.evaluate(query), N_TRIALS))
    print(f"Got {len(generated)} samples")

    n_success = 0
    apertures = []
    for i, sample in enumerate(generated):
        candidate = sample.candidate if hasattr(sample, "candidate") else sample
        candidate.graspable = cube
        candidate.end_effector = None
        candidate.approach_pose = cube._default_approach_pose()
        apertures.append(candidate.aperture)
        result = run_fixed_candidate(candidate, world, cube)
        n_success += int(result.success)
        print(f"[{i}] aperture={candidate.aperture:.4f} closing_effort={candidate.closing_effort:.4f} "
              f"-> success={result.success} slip={result.max_translation_slip:.4f}")

    print(f"\nCONDITIONED ON success=False: "
          f"{n_success}/{len(generated)} = {n_success/len(generated):.2%} real success rate")
    print(f"mean aperture: {sum(apertures)/len(apertures):.4f}, "
          f"min={min(apertures):.4f}, max={max(apertures):.4f}")
    print("For comparison: conditioned on success=True -> 100.00% (40/40), apertures 0.027-0.040")
    print("Random generator baseline (5000-sample dataset): 33.52%")


if __name__ == "__main__":
    main()

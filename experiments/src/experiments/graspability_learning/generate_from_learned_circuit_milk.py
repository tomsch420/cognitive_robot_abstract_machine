"""
Same experiment as generate_from_learned_circuit.py (condition the fitted
RelationalProbabilisticCircuit on success=True and generate candidates via
RelationalCircuitRegistry, then run each through real MuJoCo physics), now for the
milk carton instead of the cube -- milk's baseline random-sampling success rate in the
multi-geometry dataset is only 2.76% (47/1700), so this checks whether the learned
circuit can find the narrow (near-max aperture, moderate-to-high effort) region that
actually succeeds, the same way it did for the cube (33.52% baseline -> 100%).
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
from experiments.graspability_learning.domain_model import GraspCandidate, GraspTrialResult, GraspableKind
from experiments.graspability_learning.stage1_fixed_sample import build_world, run_fixed_candidate
from probabilistic_model.probabilistic_circuit.relational.rspn import (
    RelationalProbabilisticCircuit,
)

DB_PATH = "experiments/src/experiments/graspability_learning/resources/grasp_dataset_multi.sqlite"
N_TRIALS = 40
RANDOM_BASELINE = 47 / 1700  # from the corrected, independently-sampled dataset


def main():
    engine = create_engine(f"sqlite:///{DB_PATH}")
    session = sessionmaker(engine)()
    instances = list(session.scalars(select(GraspTrialResultDAO)).all())

    fitted = RelationalProbabilisticCircuit(GraspTrialResult)
    fitted.fit(instances)
    print(f"Circuit fitted on {len(instances)} instances (all 3 geometries).")

    grasp_world = build_world(GraspableKind.MILK)
    milk = grasp_world.graspable

    query = a(GraspTrialResult)(
        candidate=a(GraspCandidate)(graspable=milk, aperture=..., closing_effort=...),
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

    n_success = 0
    apertures, efforts = [], []
    for i, sample in enumerate(generated):
        candidate = sample.candidate if hasattr(sample, "candidate") else sample
        candidate.graspable = milk
        candidate.end_effector = None
        candidate.approach_pose = milk._default_approach_pose()
        candidate.graspable_kind = GraspableKind.MILK
        apertures.append(candidate.aperture)
        efforts.append(candidate.closing_effort)
        result = run_fixed_candidate(candidate, grasp_world)
        n_success += int(result.success)
        print(f"[{i}] aperture={candidate.aperture:.4f} closing_effort={candidate.closing_effort:.2f} "
              f"-> success={result.success} slip={result.max_translation_slip:.4f}")

    print(f"\nLEARNED-CIRCUIT GENERATOR (MILK): {n_success}/{len(generated)} = "
          f"{n_success / len(generated):.2%} success rate")
    print(f"RANDOM GENERATOR BASELINE (from the 5100-trial dataset): {RANDOM_BASELINE:.2%}")
    print(f"mean aperture: {sum(apertures)/len(apertures):.4f} (min={min(apertures):.4f}, max={max(apertures):.4f})")
    print(f"mean closing_effort: {sum(efforts)/len(efforts):.2f} (min={min(efforts):.2f}, max={max(efforts):.2f})")


if __name__ == "__main__":
    main()

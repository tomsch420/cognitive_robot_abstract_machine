"""
Stage 3: Cube.grasp_candidates, the real underspecified EQL generator
(aperture/closing_effort left as `...`, bounded by `.where(...)`, sampled via
ProbabilisticBackend), run through the Stage-1 physics executor.
"""
import sys
sys.path.insert(0, "experiments/src")

from experiments.graspability_learning.stage1_fixed_sample import (
    build_world, run_fixed_candidate,
)

world, cube = build_world()

for i, candidate in enumerate(cube.grasp_candidates(end_effector=None, amount=3)):
    print(f"SAMPLE {i}: aperture={candidate.aperture:.4f} closing_effort={candidate.closing_effort:.4f}")
    result = run_fixed_candidate(candidate, world, cube)
    print(f"  -> success={result.success} slip={result.max_translation_slip:.4f} contacts={result.contact_count}")

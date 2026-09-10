import sys, time
sys.path.insert(0, "experiments/src")

from experiments.graspability_learning.stage1_fixed_sample import build_world, run_fixed_candidate

t_import_done = time.time()
world, cube = build_world()

for i, candidate in enumerate(cube.grasp_candidates(end_effector=None, amount=10)):
    t0 = time.time()
    result = run_fixed_candidate(candidate, world, cube)
    t1 = time.time()
    print(f"TRIAL {i}: {t1 - t0:.2f}s success={result.success} slip={result.max_translation_slip:.4f}")

print(f"TOTAL SINCE IMPORT: {time.time() - t_import_done:.2f}s")

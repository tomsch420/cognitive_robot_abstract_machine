"""
Stage 2: the same fixed candidate as Stage 1, but expressed as an EQL Match and
evaluated through EntityQueryLanguageGenerativeBackend, then run through the exact
same Stage-1 physics executor to confirm it reproduces an identical result.
"""
import sys
sys.path.insert(0, "experiments/src")

from krrood.entity_query_language.factories import a
from krrood.entity_query_language.backends import EntityQueryLanguageGenerativeBackend

from experiments.graspability_learning.domain_model import GraspCandidate
from experiments.graspability_learning.stage1_fixed_sample import (
    build_world, run_fixed_candidate, default_grasp_pose,
)

world, cube = build_world()
approach_pose = default_grasp_pose(world)

query = a(GraspCandidate)(
    graspable=cube,
    end_effector=None,
    approach_pose=approach_pose,
    aperture=0.04,
    closing_effort=40.0,
)
print("QUERY:", query)

candidate = list(EntityQueryLanguageGenerativeBackend().evaluate(query))[0]
print("CANDIDATE FROM EQL:", candidate)

result = run_fixed_candidate(candidate, world, cube)
print("RESULT:", result)

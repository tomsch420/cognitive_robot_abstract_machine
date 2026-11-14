## Plan
- One PR (fork #40) for plan items region-graph-builds-tensorized + remove-jax-package, based on main
- Region graphs match RAT-SPN (Peharz et al. 2019): Algorithm 1 splits, cross-product partitions, factorized leaves

## Done
- Merged remove-jax-package (contains #37) with cram/main; #37/#38 closed as superseded
- Fixed region graph deviations from the paper; tests on eq. (1)/(2) counts; 1558 probabilistic_model tests pass on neem-4 (tom-probabilistic-experiments, worktree /root/region-graphs-tensorized)

- CrossEntropy loss (paper eq. 4) and fit(loss=...); removed unused ClassConditionalAverageNegativeLogLikelihood; 1561 tests pass

## Next
- Promote to cram2 (stacked-pr-maintenance) once the developer agrees; labels api-break + new-feature

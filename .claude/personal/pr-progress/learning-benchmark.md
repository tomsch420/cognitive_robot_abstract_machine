## PR #45 on the fork (learning-benchmark): Benchmark circuit learning methods against RAT-SPN, JPT and EM
- Plan: harness experiments/circuit_learning from learning-experiments, baselines only (RAT-SPN gradient descent, JPT, rustworkx EM on the RAT-SPN structure); plateau learning rate reduction with the InjectStatefulHyperparamsState fix; develop and test in the neem-4 tom-probabilistic-experiments container (worktree /root/learning-benchmark); then run the baselines on the 14 datasets
- Done: bootstrap 2026-11-17, draft PR, plan record in progress
- Next: port the harness and tests; run1 (all methods, learning-experiments worktree) is still running on neem-4, so ask before a run that competes with it

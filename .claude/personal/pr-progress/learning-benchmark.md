## PR #45 on the fork (learning-benchmark): Benchmark circuit learning methods against RAT-SPN, JPT and EM
- Plan: harness experiments/circuit_learning with the three baselines; plateau learning rate; rustworkx EM until convergence; run the baselines on the 14 datasets in the neem-4 container (worktree /root/learning-benchmark)
- Done: 53ff2791a4 pushed (harness tests 10 pass, probabilistic_model suite 1522 pass, krrood check clean); run1 stopped on the developer's word (outdated); baseline run started 2026-11-17 11:51 UTC, output /root/circuit-learning-results/learning-benchmark, log learning-benchmark.log
- Next: when the run ends, summarize it (experiments.circuit_learning.summary) into the PR description; developer to approve declaring psutil in experiments/pyproject.toml

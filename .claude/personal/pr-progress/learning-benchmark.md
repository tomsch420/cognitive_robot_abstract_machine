## PR #45 on the fork (learning-benchmark): Benchmark circuit learning methods against RAT-SPN, JPT and EM
- Plan: harness recording through ormatic; three baselines; run them on the 14 datasets in the neem-4 ROS container tom-probabilistic-experiments-ros
- Done: review round 1 at fd07a224f1, all 16 threads answered and resolved; ROS image and container built, ORM interfaces build there; baseline run started 2026-11-17 (output /root/circuit-learning-results/baselines)
- Next: when the run ends, add its results to the PR description; rustworkx EM runs out of memory (about 32 GB per slot) on the large datasets (MSNBC, POWER so far), recorded as out_of_memory; wait for review

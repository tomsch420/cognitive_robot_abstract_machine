## PR #45 on the fork (learning-benchmark): Benchmark circuit learning methods against RAT-SPN, JPT and EM
- Plan: harness recording through ormatic and queried with EQL; three baselines; run them on the 14 datasets in the neem-4 ROS container tom-probabilistic-experiments-ros
- Done: review round 2 at ce15e3d1f6, 5 of 6 threads resolved; local smoke run of 13 runs finished, none failed
- Next: developer answers the open thread on orm/model.py (keep the mapping, type mapping in generate_orm.py, or make LayeredProbabilisticCircuit a SubclassJSONSerializer); then reset /root/learning-benchmark, rebuild its ORM interfaces and restart the baseline run (still on fd07a224f1)

Plan
- Developer reviews cram2#1079 (marked ready by the developer); untuned defaults still open

Done
- Library, benchmark method, circuit-file fix (996840ca19, #1080 folded), device review comment (8eded03fb2); neem-4 30 + 143 passed
- Benchmark rerun (31595c253b, /root/circuit-learning-results/lvd-files): all 28 trainings finished; results in the PR description (test LL over supported rows + share of -inf rows, training time/memory/size, query speed, failed query runs by cause)
- LVD better on 7 datasets, within 0.035 on 4, worse on bsds300, kdd_cup_99, nyc_green_taxi; 1.5x-28x slower to train

Next
- Nothing pending unless the developer comments
- AD query runs all out of memory (13 GB circuit JSON); sparse gather bug and dense OOM are existing, not this PR

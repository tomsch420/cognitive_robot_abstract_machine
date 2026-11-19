PR cram2#1028 (draft, label bug-fix; fork draft tomsch420#49 is the same branch) - fix-table-gradient, off main. DenseProbabilityTable reads states from per-node normalized log-probabilities, so gradient descent trains tables (found in cirkit-comparison, #47).

Plan
- Single fix: normalize in log_probabilities_of_states; test that Adam learns a categorical's frequencies.

Done
- 89dd909e1f: fix and test; 786 probabilistic_model tests pass on neem-4 (worktree /root/table-gradient); krrood check clean.

Next
- Review on cram2#1028 (cram2 has no bug label). Clean merge into cram2 main at b691f5afc8. Benchmark baselines with categorical tables need a rerun once it lands.

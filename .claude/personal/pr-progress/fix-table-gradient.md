PR tomsch420#49 (draft) - fix-table-gradient, off main. DenseProbabilityTable reads states from per-node normalized log-probabilities, so gradient descent trains tables (found in cirkit-comparison, #47).

Plan
- Single fix: normalize in log_probabilities_of_states; test that Adam learns a categorical's frequencies.

Done
- 89dd909e1f: fix and test; 786 probabilistic_model tests pass on neem-4 (worktree /root/table-gradient); krrood check clean.

Next
- Review. The repository lacks a bug-fix label (bug is set). Benchmark baselines with categorical tables need a rerun once it lands.

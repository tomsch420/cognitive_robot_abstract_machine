Plan
- Put the LVD vs HCLT benchmark results into the cram2#1079 description once the neem-4 rerun ends (developer asked 2026-11-26)
- Developer reviews; untuned defaults still open

Done
- Library + benchmark method (a9465dfc80, 66b51145b6, 58c946c4b4); tests 27 + 138 passed on neem-4
- First run (/root/circuit-learning-results/lvd) stopped: AD lost both methods (SQLite INT_MAX on the circuit); fixed in cram2#1080 (circuit files, kept)
- Rerun started 2026-11-26 ~23:55 CET: worktree /root/lvd-rerun (= #1079 + #1080 merged, 31595c253b), launcher /root/lvd-rerun-run.sh, output /root/circuit-learning-results/lvd-files (log lvd-files.log), both methods, 14 datasets, first settings only
- Summary: docker exec -w /root/circuit-learning-results tom-probabilistic-experiments-ros python3 summarize_lvd.py lvd-files/results.sqlite
- First-run numbers (test LL, LVD vs HCLT): plants -13.15 vs -14.31, nltcs -6.02 vs -6.04, msnbc -6.18 vs -6.18; LVD trains slower

Next
- Note in description: msnbc numpy sparse queries fail in edge_matrix.gather (also HCLT, existing bug); plants dense queries out of memory (17.4 TiB, dense edges); ignore CUDA_ERROR_NO_DEVICE in CPU query logs; results need #1080 for AD

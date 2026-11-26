Plan
- Put the LVD vs HCLT benchmark results into the cram2#1079 description once the neem-4 run ends (developer asked 2026-11-26)
- Developer reviews; untuned defaults still open

Done
- Library + benchmark method (a9465dfc80, 66b51145b6, 58c946c4b4); tests 27 + 138 passed on neem-4
- Run started 2026-11-26 23:10 CET: worktree /root/latent-variable-distillation, launcher /root/latent-variable-distillation-run.sh, output /root/circuit-learning-results/lvd (log lvd.log), both methods, 14 datasets, first settings only
- Summary: docker exec -w /root/circuit-learning-results tom-probabilistic-experiments-ros python3 summarize_lvd.py lvd/results.sqlite
- First results (test LL, LVD vs HCLT): plants -13.15 vs -14.31, nltcs -6.02 vs -6.04, msnbc -6.18 vs -6.18; LVD trains slower (autoencoder)

Next
- AD: result + circuit are one transaction (runner.py:191), so the SQLite overflow loses all AD numbers and its queries; offered: one-off script printing AD train/test LL, time, size for both methods (no storage); storage fix would be its own PR
- Note in description: msnbc numpy sparse queries fail in edge_matrix.gather (also plain HCLT, existing bug); AD HCLT result not stored (SQLite OverflowError: string longer than INT_MAX), LVD on AD likely same; ignore CUDA_ERROR_NO_DEVICE in CPU query logs

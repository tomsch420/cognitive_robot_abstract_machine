## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- Optimize what profiling found, without slowing gradient descent; fill the PR #55 tables; name labels api-break, new-feature to the user.
- No further benchmark runs (user, 2026-11-23). cirkit left out of the EM comparison (user, 2026-11-22).

### Done
- Commits through c793a60d2a pushed (bases, tensorized EM jax/torch, tables, experiments, fused update, ProcessorCache, rows_per_chunk scan).
- Benchmarks: results in /root/circuit-learning-results/tensorized-em/{hepmass-gpu,hepmass-cpu-share,rerun}. First CPU wave contention-inflated; isolated jax CPU reruns: full-batch EM f32 5.52 / f64 10.84 s, online f32 5.73 / f64 12.20 s per epoch; jax GPU f64 EM 1.539 s.
- Profiling: EM step costs the same as GD (GPU kernels 0.567 vs 0.573 ms; torch CPU equal within noise). GPU epoch is host-bound by batches_of_epoch (0.18 s = all steps); gathering rows inside the compiled jax step: 0.35 -> 0.19 s per epoch for EM and GD. Epoch average includes first-epoch compilation (~0.1 s), as in #1046.

### Next
- If approved: MiniBatches yields row indices, MiniBatchTraining gathers by default, jax training gathers inside its compiled step (single device); test-driven; measure EM and GD epochs.
- Optional: chunking sweep (em-chunk-sweep.sh, ~15 min) or drop rows_per_chunk.
- Fill PR #55 tables (note contention, compilation in epoch average, cirkit wording).

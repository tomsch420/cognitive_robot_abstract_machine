## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- Benchmark HEPMASS 100 epochs / 1 h: our EM (full-batch, online) vs Einsum Networks' EM vs GD rerun vs host EM; GPU sequential, CPU 25-core shares in two waves.
- Fill the measurements tables in the PR description; name labels (api-break, new-feature) to the user.

### Done
- Host EM baseline: 42 s / 10k, 677 s / 100k, 3759 s / 315k rows per iteration.
- Shared bases, keyword-only learners (25269e1630); tensorized EM jax/torch with host-agreement tests (bf5bc2cd7a); dense probability tables (db16270593); experiments methods em + online (c08bc0739c); fused update step (7a6aa41560).
- Step profile: EM step kernel-bound (GPU kernel 0.566 ms = wall 0.567 ms); GD unchanged (0.575 jax GPU, 3.84 torch GPU).
- Batch size: jax CPU float32 full-batch EM 3.0 s/epoch at 4096 vs 4.9 at 1024, cliff at 8192 / float64 4096; not adopted (default 1024).

### Next
- Collect /root/circuit-learning-results/tensorized-em/hepmass-{gpu,cpu-share}/*.json and update PR #55 description.

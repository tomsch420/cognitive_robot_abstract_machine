## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- Wire tensorized EM into the circuit-learning experiments (full-batch and online, jax/torch, cpu/gpu, float32/float64, block edges).
- Profile and optimize EM like #1046 did (warm synced steps, jax profiler, CUDA graphs incl. the update, jax CPU device split); check gradient descent does not slow down.
- Benchmark HEPMASS 100 epochs: our EM vs Einsum Networks' EM vs GD (#1046) vs host EM; CPU runs on 25-core shares.
- Fill the measurements tables in the PR description.

### Done
- Host EM baseline: 42 s / 10k rows, 677 s / 100k rows, 3759 s / full 315k rows per iteration.
- Refactor: shared CircuitParameterLearning/Training bases, jax and torch bases; learners keyword-only (25269e1630).
- Tensorized EM (jax, torch) with tests vs host EM, online EM, CUDA graphs, split batches (bf5bc2cd7a).

### Next
- Experiments method for tensorized EM; harness /root/hepmass_epoch_speed.py with EM methods; Einsum Networks EM probe.

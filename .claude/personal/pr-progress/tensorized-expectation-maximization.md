## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- Finish the PR: decide on ProcessorCache (unused now); name labels api-break, new-feature to the user.
- No further benchmark runs (user, 2026-11-23). cirkit left out of the EM comparison (user, 2026-11-22).

### Done
- Pushed through b3336043e1: tensorized EM jax/torch (sum, Gaussian, discrete), experiments, fused update, ProcessorCache, chunk scan added then dropped (c015dc747a: chunked = plain batch of chunk size), rows gathered inside the compiled jax step (jax GPU epoch loop 0.37 -> 0.21 s EM, 0.36 -> 0.23 s GD; torch and split batches unchanged).
- Benchmarks and profile written into the PR #55 description (GPU table, isolated jax CPU table, contended CPU wave table, host EM, cirkit wording, profile).
- 148 tests pass; krrood format/check clean.

### Next
- Ask user whether ProcessorCache (krrood_compute_resources, 749ceb3dd9) can be removed: only its tests use it, since cache size did not predict the best CPU batch size (4x off).

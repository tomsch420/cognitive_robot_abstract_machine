## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- PR is complete; labels api-break, new-feature named to the user.
- No further benchmark runs (user, 2026-11-23). cirkit left out of the EM comparison (user, 2026-11-22).

### Done
- Pushed through b3336043e1: tensorized EM jax/torch (sum, Gaussian, discrete), experiments, fused update, ProcessorCache added then removed (c543a066dd, user agreed), chunk scan added then dropped (c015dc747a: chunked = plain batch of chunk size), rows gathered inside the compiled jax step (jax GPU epoch loop 0.37 -> 0.21 s EM, 0.36 -> 0.23 s GD; torch and split batches unchanged).
- Benchmarks and profile written into the PR #55 description (GPU table, isolated jax CPU table, contended CPU wave table, host EM, cirkit wording, profile).
- 148 tests pass; krrood format/check clean.

### Next
- Nothing open; wait for review or further instructions.

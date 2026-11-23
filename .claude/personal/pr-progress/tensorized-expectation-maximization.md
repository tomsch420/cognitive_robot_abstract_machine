## PR progress: tensorized-expectation-maximization (fork PR #55, stacked on cram2#1046)

### Plan
- PR is complete; labels api-break, new-feature named to the user.
- No further benchmark runs (user, 2026-11-23). cirkit left out of the EM comparison (user, 2026-11-22).

### Done
- Pushed through b3336043e1: tensorized EM jax/torch (sum, Gaussian, discrete), experiments, fused update, ProcessorCache added then removed (c543a066dd, user agreed), chunk scan added then dropped (c015dc747a: chunked = plain batch of chunk size), rows gathered inside the compiled jax step (jax GPU epoch loop 0.37 -> 0.21 s EM, 0.36 -> 0.23 s GD; torch and split batches unchanged).
- Benchmarks and profile written into the PR #55 description (GPU table, isolated jax CPU table, contended CPU wave table, host EM, cirkit wording, profile).
- Probes: torch GPU kernel-bound; gather along expanded index (d860749c13) -> torch GPU step 4.02 -> 2.87 ms EM, 3.84 -> 2.71 GD, jax and torch CPU unchanged. torch CPU threads: 25 stays. Compilation ~5 s once per run, not worth changing.
- 1,010 tests pass; krrood format/check clean; PR description updated.
- Fixed the intermittent CUDA-graph capture failure (0d8d61ea67): recorded steps keyed by the unbound step function (no training reference cycle), GC paused during recording; tests in test_recorded_training_steps.py failed first, 109 GPU tests pass. Upstream: commented a torch-only repro on pytorch#199117 (user approved).

### Next
- Nothing open; wait for review or further instructions.

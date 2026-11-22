## cram2#1046 Train block-edge circuits up to 10x faster with jax and 4x with torch (branch block-edges-without-gathers)

**Plan**
- All measured training optimizations in one PR: gather-free block reduction, sums as matrix products, products as cross products, gradient stop on log-sum-exp shifts, one loss sync per epoch, torch steps recorded as CUDA graphs.

**Done**
- 3118ba8097 pushed (on cram2 main incl. #1040): torch GPU step 14.5 -> 3.84 ms (f32) with records_cuda_graphs; jax unchanged (checked before/after on GPU and CPU); probabilistic_model suite 1867 passed, 1 xfailed; experiments circuit_learning tests pass; krrood check clean; PR description updated.

**Next**
- Developer discusses the api-break with the probabilistic_model code owners.
- Rerun the torch GPU rows of the HEPMASS comparison with CUDA graphs (the running comparison used f8d07cc0f3).
- Open: GradientStop and NormalDistributionFunctions each pick their backend class (sharing a base needs the monkeypatch test changed); CUDA graphs are opt-in until truncated Gaussian, binomial and probability-table layers stop copying from the host.

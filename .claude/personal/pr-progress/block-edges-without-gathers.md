## cram2#1046 Train block-edge circuits up to 10x faster (branch block-edges-without-gathers, on top of #1040)

**Plan**
- All measured optimizations in one PR (developer's choice): gather-free block reduction, sums as matrix products, products as cross products, gradient stop on log-sum-exp shifts, one loss sync per epoch.

**Done**
- f8d07cc0f3 pushed: HEPMASS jax epoch CPU 240 -> 24 s, GPU 2.0 -> 0.18 s; probabilistic_model suite 1858 passed, 1 xfailed; krrood format/check clean; PR description updated.
- Tried and dropped: donating the training state to jax.jit (no gain).

**Next**
- Developer discusses the api-break (keyword-only layer methods, step_on_batch returning arrays) with the probabilistic_model code owners.
- Rebase once #1040 merges.
- Open question for the developer: NormalDistributionFunctions and GradientStop each pick their backend class; sharing a base would need the monkeypatch in test_normal_distribution_functions changed.

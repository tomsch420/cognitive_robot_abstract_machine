PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. Measurements paused by the developer (2026-11-19); the neem-4 cirkit run was left running.

Plan
- Developer to decide: fix the dense table parameterization in probabilistic_model (own bug-fix PR off main), and whether folding layers becomes a plan item.

Done
- 36fdf76e53: cirkit's log sigma floored at GaussianLayer.minimum_scale like the baseline (float32 NaN on MINIBOONE/NYC/KDD/ALLSTATE); test, 15 tests pass on neem-4.
- Log-likelihood gap found: DenseProbabilityTable reads raw log-probabilities and gradient descent renormalizes after each Adam step, so the gradient only raises observed states and Adam's per-coordinate steps cancel under renormalization. Normalizing in the forward pass (log-softmax, as cirkit) makes jax gradient descent on NLTCS reach exactly cirkit's -6.101 in 50 epochs (was -8.799).
- Speed gap found: the layered NLTCS RAT-SPN runs 371 layers and 14653 CUDA kernels per likelihood call (298 ms for 3236 rows); cirkit folds it into 6 layers and 123 kernels (2.6 ms). Encoding and transfer are ~1.5 ms.
- Scripts: neem-4 /root/{likelihood_profile,parameter_movement}.py, worktree /root/cirkit-scale-floor.

Next
- The developer's call on the two fixes above.

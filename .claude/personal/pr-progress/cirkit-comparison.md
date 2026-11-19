PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. libcirkit 0.3.1 installed --no-deps; queries timed in cirkit. Bug fix #48 (torch table parameter count) merged in.

Plan
- Decide with the developer: restart the neem-4 cirkit run with 36fdf76e53 (scale floor), or rerun only its failed runs. Runs recorded so far are unfloored.
- Confirm on GPU float32 that NYC/KDD/MINIBOONE no longer end on a NaN scale.
- Add the compare_cirkit.py summary of the full run to the PR and the page.
- Check why jax gradient descent converges at -8.80 on NLTCS after 50 epochs while cirkit reaches -6.02.

Done
- a6170aa9fd method, conversion, cirkit timing; 9fe7901ee9 merges #48; 92c0ba9c43 one-value symbolic columns.
- 36fdf76e53: cirkit's log sigma projected to log(GaussianLayer.minimum_scale) after every step, read-back projected; test fails without it. 15 circuit_learning tests pass on neem-4; krrood check clean.
- Merge probe clean against main at 92c0ba9c43; no checks run on the fork; no reviews.
- Test worktree neem-4 .../cram-probabilistic-experiments-home/cirkit-scale-floor (/root/cirkit-scale-floor), env script /root/scale-floor-env.sh.

Next
- The developer's answer on the running benchmark.

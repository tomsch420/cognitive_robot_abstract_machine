PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. libcirkit 0.3.1 installed --no-deps; queries timed in cirkit. Bug fix #48 (torch table parameter count) merged in.

Plan
- Fix the float32 cirkit training runs that crash on a NaN Gaussian scale (MINIBOONE, NYC_GREEN_TAXI, KDD_CUP_99, ALLSTATE, read 2026-11-19 at 92c0ba9c43): the baseline keeps every scale at least GaussianLayer.minimum_scale, cirkit's exp of the log scale has no floor. Prove it on NYC in neem-4 worktree cirkit-scale-floor, test first, then floor the scale as the baseline does.
- Rerun the float32 cirkit runs of those datasets after the main run (cirkit-rerun-failed.sh covers failed runs).
- Full neem-4 run (tom-probabilistic-experiments-ros, /root/circuit-learning-results/cirkit, note cirkit-NOTES.md); add the compare_cirkit.py summary to the PR and the page.
- Check why jax gradient descent converges at -8.80 on NLTCS after 50 epochs while cirkit reaches -6.02.

Done
- a6170aa9fd method, conversion, cirkit-native likelihood timing; 9fe7901ee9 merges #48; 92c0ba9c43 one-value symbolic columns.
- Merge probe against main clean at 92c0ba9c43; no checks ran on the fork, no reviews.
- Comparison page: https://claude.ai/artifact/Xs99hE6QMgxxyHmJHoMRXX

Next
- Confirm the scale collapse on NYC (diagnose.py), then the fix.

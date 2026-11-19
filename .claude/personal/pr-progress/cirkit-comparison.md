PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. libcirkit 0.3.1 installed --no-deps; queries timed in cirkit. Bug fix #48 (torch table parameter count) merged in.

Plan
- Full neem-4 run (started 2026-11-19, tom-probabilistic-experiments-ros, /root/circuit-learning-results/cirkit, note cirkit-NOTES.md); add the compare_cirkit.py summary to the PR and the page.
- Check why jax gradient descent converges at -8.80 on NLTCS after 50 epochs while cirkit reaches -6.02.

Done
- a6170aa9fd method, conversion, cirkit-native likelihood timing; 9fe7901ee9 merges #48.
- ORM interfaces build on neem-4; smoke run on NLTCS finishes all runs.
- Comparison page: https://claude.ai/artifact/Xs99hE6QMgxxyHmJHoMRXX

Next
- Read the run with compare_cirkit.py once it ends; update PR and page.

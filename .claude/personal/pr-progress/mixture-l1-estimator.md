PR #769 (draft): https://github.com/cram2/cognitive_robot_abstract_machine/pull/769

Plan: replace MonteCarloEstimator's L1 estimators with the importance sampling
estimate under the mixture (p + q) / 2, averaging 2|p - q| / (p + q).

Done:
- Failing test first (nearly identical Gaussians), then the new l1_metric;
  removed l1_metric_but_with_uniform_measure and the tolerance parameter.
- test/probabilistic_model_test/test_monte_carlo_estimator.py: 3 tests pass.
- Pushed to origin, draft PR opened.
- Review comment: references now cited via sphinxcontrib-bibtex
  (:cite:p:, entries in probabilistic_model/doc/references.bib); replied, resolved.
- User preferred [0, 1]: renamed l1_metric -> total_variation_distance
  (half of L1); tests, PR title and description updated.

Next:
- docformatter missing locally; run scripts/format_docstrings.py once installed.
- Wait for CI and review; handle only when asked.

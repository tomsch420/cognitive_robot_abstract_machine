PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. libcirkit 0.3.1 approved, installed --no-deps; queries timed in cirkit itself.

Plan
- Converter LayeredProbabilisticCircuit <-> cirkit, tests first (equal log-likelihoods).
- LearningMethod.CIRKIT: our RAT-SPN region graph, cirkit torch training (Adam, same mini-batches, convergence).
- cirkit-native likelihood, marginal and sampling timing; conditioning/truncation not answered.
- CI installs libcirkit --no-deps; feature comparison in the PR description.

Done
- Branch, draft PR, plan record.

Next
- Read the layered circuit's layer types to map them onto cirkit layers.

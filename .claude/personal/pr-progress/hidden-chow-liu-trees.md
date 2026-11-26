Plan
- Developer confirms or corrects the prototype defaults (12 hidden states, 10 bins, init [0.1, 1], mini-batch 512 rows / step 0.1) and the deviations listed in the PR description
- Then mark ready on request

Done
- Library: mutual information + ChowLiuTree + HiddenChowLiuTree (random init, numpy host circuit) in learning/structure (a81e208534)
- Benchmark: HiddenChowLiuTreeByExpectationMaximization (mini-batch then full-batch EM, jax/torch), shared ExpectationMaximizationPhase and TrainingContext.train_until_converged, ConvergenceHistory.from_consecutive (7e43eab5e4)
- Tests green on neem-4 at 7e43eab5e4 (circuit_learning_test 19 passed; test_learning + training-until-convergence 146 passed); PR description updated; labels new-feature, maintenance

Next
- Wait for the developer's answers on the open questions
- Plan site dispatch setup skipped by the user (plan_site.publication not run)

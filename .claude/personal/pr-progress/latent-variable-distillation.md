Plan
- Developer reviews draft cram2#1079; untuned defaults (autoencoder size, 10 epochs, pseudocount 0.1, clustering_rows 10000) and a benchmark run still open

Done
- Dependency: none new (torch.nn + scikit-learn), porting the developer's prototype from fork branch circuit-learning-methods
- CountedParameters + InitialParameters in HCLT (a9465dfc80); LatentVariableDistillation module (66b51145b6); benchmark method + ComputationSetting.torch_device (58c946c4b4)
- neem-4 worktree /root/latent-variable-distillation (env script alongside): circuit_learning_test 27 passed, test_learning 138 passed at 58c946c4b4
- PR description final; labels new-feature + api-break (RandomInitialParameters.transition_weights gains child)

Next
- Nothing pending unless the developer comments
- Local ORM regeneration fails in coraplex (CouldNotResolveType PlanExecutor); experiments tests run on neem-4 instead

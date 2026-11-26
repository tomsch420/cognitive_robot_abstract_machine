Plan
- HCLT: abstract InitialParameters; RandomInitialParameters + new CountedParameters (pseudocount); build_circuit takes the parameters
- learning/structure/latent_variable_distillation.py: torch.nn masked autoencoder + MiniBatchKMeans, ported from fork branch circuit-learning-methods
- experiments: LatentVariableDistillationByExpectationMaximization, jax+torch, same EM phases as HCLT
- Tests first for each; run on neem-4 (tom-probabilistic-experiments-ros)

Done
- Dependency decided: none new (torch extra + scikit-learn), per developer's neem-4 prototype
- Branch, draft cram2#1079 (new-feature), plan item in progress

Next
- Write CountedParameters tests and port

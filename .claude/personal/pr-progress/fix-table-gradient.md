PR cram2#1028 (ready for review; labels bug-fix, new-feature) - fix-table-gradient, from the fork; fork draft tomsch420#49 is the same branch. Dense tables trained through their normalization, and jax log-likelihoods compiled.

Plan
- 89dd909e1f: DenseProbabilityTable reads per-node normalized log-probabilities, so gradient descent trains tables.
- b8c1fd0f88: probabilistic_model.jax_log_likelihood.CompiledLogLikelihood; the benchmark's ChunkedLogLikelihoodEvaluation uses it for jax circuits, last chunk padded.

Done
- 786 probabilistic_model tests and 8 circuit_learning tests pass on neem-4 (worktree /root/table-gradient); krrood check clean; clean merge into cram2 main at b691f5afc8.
- NLTCS jax GPU likelihood 3276 -> 562 k rows/s (first call 8.2 s).

Next
- Review on cram2#1028. Torch compile needs device-resident index arrays (follow-up). Benchmark baselines need a rerun once it lands.

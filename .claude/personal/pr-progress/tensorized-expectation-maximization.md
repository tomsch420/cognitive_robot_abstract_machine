<!-- BEGIN-PR-PROGRESS -->
## PR progress: cram2#1050 (tensorized-expectation-maximization)

### Plan
- Review of 2026-11-23 answered; wait for the reviewer on the two open questions.

### Done
- f36c0e1d53: EM rules out of layers (LayerMaximization[LayerType]), jax/torch trainings deduplicated, kw-only jax steps, CompiledStep instead of inner function.
- 97569c74eb: learning/ split into structure/ and parameters/{gradient_descent,expectation_maximization}/, exceptions moved next to raisers, all imports + core list updated.
- Tests green at 97569c74eb (pm 1956, experiments 15, krrood 198); krrood check --since f2ffe7c071 clean. Threads replied; 4 resolved, 2 open (override docstring in methods.py, empty __post_init__); description updated.

### Next
- Open threads: methods.py override docstring, CircuitParameterLearning.__post_init__ - act on the reviewer's answer.
<!-- END-PR-PROGRESS -->

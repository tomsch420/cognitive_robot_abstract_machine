Plan
- Developer reviews the draft; mini-batch phase defaults (0.1 step / 512 rows, paper states none) still open

Done
- Library, benchmark method, review rounds, estimators, step-memory fixes (#1073/#1075 folded in and closed)
- Review round 3: UniformWeightRange (7328649e69), mixture_layer inlined, binned MI counted in chunks (d46ce4116b), rows_per_step measures the training learner via ParameterLearnerBuilder (1c24fd4df3), nearest neighbour maximum_rows=10000 chosen by a 7-dataset subsample experiment (98378d4fea); all threads replied and resolved
- Edge speed answered: dense fastest for small HCLT with jax, block fastest with torch, dense out of memory for folded RAT-SPN and AD; sparse compile hangs (developer: leave it)
- neem-4 circuit_learning_test 22 passed at 98378d4fea; PR description updated

Next
- Nothing pending unless the developer comments
- Plan site dispatch setup skipped by the user (plan_site.publication not run)

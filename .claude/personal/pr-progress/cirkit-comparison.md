PR tomsch420#47 (draft) - cirkit-comparison, plan circuit-learning-methods. libcirkit 0.3.1 approved, installed --no-deps; queries timed in cirkit itself.

Plan
- Run the cirkit method on neem-4 (install libcirkit --no-deps in the container), add its summary to the PR.
- Developer reviews #47; mark ready only when told.

Done
- a6170aa9fd: layer-by-layer cirkit conversion with read-back through the compiler state, RAT-SPN cirkit method (Adam, baseline's batches/plateau/convergence), cirkit-native likelihood timing, per-method query settings/measurement, CI install step; 13 tests pass, krrood check clean.
- Found: cirkit 0.3.1 cannot sample Kronecker (RAT-SPN) circuits, nor sample in float64; sampling recorded as not answered.

Next
- neem-4 benchmark run of RAT-SPN cirkit vs gradient descent.

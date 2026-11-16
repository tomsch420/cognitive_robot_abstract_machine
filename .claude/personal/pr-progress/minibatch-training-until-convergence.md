PR tomsch420#43 (draft), upstream cram2#980 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Developer reviews cram2#980; mark ready only when told.

Done
- Convergence, MiniBatches, MiniBatchTraining interface, minimize_until_converged on both trainers.
- Batch size from free memory (8792a76e7c): DeviceMemory (host, jax, torch), StepMemory from two probes (jax: XLA analysis; torch: CUDA peak; torch CPU raises), MiniBatches.fitting_in_memory.
- All review threads on cram2#980 answered and resolved. 1,590 tests pass with the A40; krrood check clean.

Next
- Start learning-benchmark stacked on this branch; reuse DeviceMemory for its peak-memory column.

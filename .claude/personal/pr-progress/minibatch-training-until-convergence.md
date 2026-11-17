PR tomsch420#43 (draft), upstream cram2#980 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Developer reviews cram2#980; mark ready only when told.

Done
- Convergence, MiniBatches, MiniBatchTraining interface, minimize_until_converged on both trainers.
- Batch size from free memory (8792a76e7c): DeviceMemory (host, jax, torch), StepMemory from two probes (jax: XLA analysis; torch: CUDA peak; torch CPU raises), MiniBatches.fitting_in_memory.
- Renamed (145ed6a546): abstract GradientDescent, JaxGradientDescent, TorchGradientDescent. Merged cram/main and fixed the moved classproperty import (214bb5f035); 1,590 tests pass.
- Second review round (dc1ca44502): both trainers subclass the shared base (shared minimize, minimize_until_converged, step_memory); memory_file/memory_field renames. Threads answered and resolved; 1,590 tests pass; krrood check clean.

Next
- Start learning-benchmark stacked on this branch; reuse DeviceMemory for its peak-memory column.

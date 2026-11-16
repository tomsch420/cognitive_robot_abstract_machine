PR tomsch420#43 (draft), upstream cram2#980 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Developer reviews cram2#980; mark ready only when told.

Done
- Convergence, MiniBatches, abstract MiniBatchTraining with GradientDescentTraining and TorchGradientDescentTraining; minimize_until_converged on both trainers.
- First review answered at 9c6e61cb38 (no inner functions, interface instead of callables, cheaper tests); 4 threads replied to and resolved. 1,575 tests pass, krrood check clean.

Next
- Start learning-benchmark stacked on this branch.

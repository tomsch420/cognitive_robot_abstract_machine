PR tomsch420#43 (draft), upstream cram2#980 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Open: thread on cram2#980 asking whether MiniBatches should get a batch-size constructor from system memory, which way, and in which PR.
- Developer reviews cram2#980; mark ready only when told.

Done
- Convergence, MiniBatches, abstract MiniBatchTraining with GradientDescentTraining and TorchGradientDescentTraining; minimize_until_converged on both trainers.
- Reviews answered at b5ef3f0a1b: no inner functions, MiniBatchTraining interface, cheaper tests, trained_root, MiniBatches as a parameter. 1,575 tests pass, krrood check clean.

Next
- Start learning-benchmark stacked on this branch.

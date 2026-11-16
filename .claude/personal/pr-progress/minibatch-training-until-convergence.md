PR tomsch420#43 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Backend-free Convergence criterion (tolerance 0.001, patience 3, at most 1,000 epochs) in learning/convergence.py.
- minimize_until_converged on GradientDescent and TorchGradientDescent: shuffled mini-batches, returns trained root, epochs, converged.
- Tests first: stops at convergence, stops at the epoch limit, raises the log-likelihood on jax and torch.

Done
- Branch, draft PR #43, plan record.

Next
- Write the tests, then the implementation; krrood format/check; probabilistic_model tests.

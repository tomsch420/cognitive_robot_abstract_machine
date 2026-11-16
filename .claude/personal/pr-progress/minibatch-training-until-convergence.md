PR tomsch420#43 (draft) - mini-batch gradient descent until convergence, plan circuit-learning-methods / minibatch-training-until-convergence. learning-benchmark stacks on it.

Plan
- Developer reviews the draft; mark ready only when told.

Done
- Convergence, ConvergenceHistory, MiniBatches (learning/training_schedule.py); TrainingRun and minimize_until_converged on GradientDescent and TorchGradientDescent.
- Tests: 1,576 of test/probabilistic_model_test pass on neem-4; krrood check clean. Pushed; description updated.

Next
- Start learning-benchmark stacked on this branch.

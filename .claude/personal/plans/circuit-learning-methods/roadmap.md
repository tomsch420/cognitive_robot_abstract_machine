# State-of-the-art learning methods for probabilistic circuits

- Every method reports test log-likelihood, accuracy where it classifies, and training time on the shared benchmark, against RAT-SPN and JPT.
- Implement each method as published; name every deviation from the paper in the pull request description.
- Methods train the tensorized layered circuits on any array backend wherever the method allows; rustworkx only where structure search needs it.
- A new dependency, such as a deep generative model for distillation, is approved by the developer before it is added.
- The survey may add, reorder or drop candidate items; every such change is announced on the tracking pull request.

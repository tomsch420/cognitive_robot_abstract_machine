# State-of-the-art learning methods for probabilistic circuits

- Every method reports, on the shared benchmark against RAT-SPN and JPT: training time and resources, train and test log-likelihood, the speed of likelihood, conditioning, truncation and sampling (relational grounding once a relational dataset joins), model size, and the list of queries, causal ones included, it answers exactly.
- Implement each method as published; name every deviation from the paper in the pull request description.
- Methods train the tensorized layered circuits on any array backend wherever the method allows; rustworkx only where structure search needs it.
- A new dependency, such as a deep generative model for distillation, is approved by the developer before it is added.
- Every change to the candidate items is announced on the tracking issue.

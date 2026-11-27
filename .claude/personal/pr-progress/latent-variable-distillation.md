## PR cram2#1079 latent-variable-distillation

### Plan
- Address the 2026-11-27 review (Narenvasant, with tomsch420's instructions)

### Done
- 91fcd7adb7: counted Gaussian variance from deviations around each state's mean (cancellation for large values), test added
- b59739d962: TransformerArchitecture bundles width/layers/heads/feedforward_width; trained_autoencoder seeds inside torch.random.fork_rng (untrained_autoencoder removed); EmbeddingBatch replaces the tuple
- neem-4 at b59739d962: circuit_learning_test 30 passed, test_learning 143 passed; CUDA random state kept on the A40
- Four threads answered and resolved; description updated

### Next
- CI on b59739d962 not read yet; benchmark not rerun after the variance fix
- Changes requested review by Narenvasant still stands until they re-review

# Folded layered circuits: standing rules

- The layered circuit stays the only representation: no compiled or folded copy beside it, and each layer type's numerical code is written once.
- Every step keeps every query and every existing test passing; no query loses support or falls back to another representation.
- A folded circuit answers every query exactly as its unfolded circuit does, tested on numpy, jax and torch.
- Folding works through the Array API on every backend, eagerly, without depending on a compiler.

PR tomsch420#54 (draft) - folding-pass, plan folded-layered-circuits, base per-node-scopes (#53) with folded-edge-matrix (#52) merged. LayerFolding and LayeredProbabilisticCircuit.folded.

Plan
- Done: folding, tests, gain measured (NLTCS, MINIBOONE): torch GPU 26-39x, jax eager 21-37x, jit compile 15-28x, jit 1.15-1.3x, numpy none.

Done
- cd16c41081; 17 folding tests pass on neem-4 (worktree /root/folding-pass, script /root/folding_gain.py).

Next
- Full probabilistic_model suite on neem-4 at cd16c41081 (running).
- Developer decision: RegionGraph builds folded circuits by default?
- Torch on GPU: keep index arrays on the device (outside the plan).

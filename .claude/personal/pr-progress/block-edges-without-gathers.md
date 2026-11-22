<!-- BEGIN-PR-PROGRESS -->
## cram2#1046 Reduce the children of block edges without a gather per edge (branch block-edges-without-gathers, on top of #1040)

### Plan
- Reprofile the jax CPU and GPU training step on the branch and measure the next improvements.

### Done
- EdgeMatrix.reduce_child_values_by_row + EdgeWeighting; BlockEdgeMatrix reads each block's children once (reshape when contiguous), skips no-op fill/reorder; sum/product layers use it, keyword-only.
- Tests (block vs sparse, jax gradients, every backend); probabilistic_model suite 1845 passed; krrood format/check clean.
- HEPMASS step: CPU 374 -> 206 ms, GPU 6.4 -> 2.4 ms. Draft PR opened with api-break + new-feature.

### Next
- Reprofile; candidates: cross-product layers (prototype fix B), remaining logsumexp/reduction cost.
<!-- END-PR-PROGRESS -->

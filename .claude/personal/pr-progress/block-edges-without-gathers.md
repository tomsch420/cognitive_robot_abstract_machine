<!-- BEGIN-PR-PROGRESS -->
## cram2#1046 Reduce the children of block edges without a gather per edge (branch block-edges-without-gathers, on top of #1040)

### Plan
- Decide with the developer which measured follow-ups go into #1046 or later PRs: M (sums as matrix product), B (cross-product edges), S (read loss once per epoch).

### Done
- EdgeMatrix.reduce_child_values_by_row + EdgeWeighting; BlockEdgeMatrix reads each block's children once (reshape when contiguous), skips no-op fill/reorder; sum/product layers use it, keyword-only. Tests; suite 1845 passed; krrood clean. Draft PR open (api-break, new-feature).
- HEPMASS step on the branch: CPU 374 -> 206 ms, GPU 6.4 -> 2.4 ms.
- Reprofiled; prototypes on the branch (/root/jax_fixes2.py, FIXES2=M,B): M CPU 205 -> 93 ms, M+B CPU 75 ms / GPU 1.6 ms; M changes losses at about 1e-7 relative. S: CPU 75.7 -> 58.5 ms, GPU 1.29 -> 1.02 ms.

### Next
- Implement whichever of M, B, S the developer picks, with tests and before/after numbers.
<!-- END-PR-PROGRESS -->

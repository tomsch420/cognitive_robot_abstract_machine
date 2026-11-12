## PR #945 (inner-layers-any-backend)
- Plan: abstract EdgeMatrix (SparseEdgeMatrix, DenseEdgeMatrix) for sum and product layers, folded into this PR; sum log weights stored in the edge matrix's storage; structural queries rebuild through type(self.edges); remove per call namespace overhead; benchmark again
- Done: backend neutral inner layers (cfd96778e3), 1550 tests pass; benchmark: RAT-SPN inference +12 %, truncation +4 %; edit worktree inner-layers-edges on neem-4
- Next: write tests for both edge matrices, then implement

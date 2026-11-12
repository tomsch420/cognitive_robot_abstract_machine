## PR #945 (inner-layers-any-backend)
- Plan: inner layers and circuit log likelihood on numpy and jax through xp; benchmark against main
- Done: RowReduction enum, backend neutral RowGroupedSparsity, SumLayer as HasParameterArrays, product layer on the backend of its children, circuit keeps backend events (cfd96778e3); 1550 tests pass; old reduce_by_row tests switched to RowReduction with the developer's OK; api-break label
- Next: benchmark result into the PR description, then review

## PR #945 (inner-layers-any-backend)
- Plan: inner layers on any backend through an abstract EdgeMatrix (sparse, dense), folded into this PR; benchmark against main
- Done: EdgeMatrix with SparseEdgeMatrix and DenseEdgeMatrix, both layers written against it, store_edges_as, row_grouped_sparse_array removed (45e3d0c953); 1571 tests pass; gathers by integer indexing (take was 32 % slower)
- Next: benchmark result into the PR; then the array_types cleanup the developer asked about (value aliases on BackendArray), once they confirm

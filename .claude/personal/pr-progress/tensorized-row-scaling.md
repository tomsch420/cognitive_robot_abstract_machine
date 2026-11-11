## Plan
- Decide with the user: remove embedded_logsumexp (test only now); stream the linear CDF and moment reductions too or leave them
- Review of PR 930

## Done
- RowGroupedSparseArray.logsumexp_by_row streams logaddexp over entry slots without padding; SumLayer uses it
- Tests for uneven rows, all -inf, rows without entries; full probabilistic_model suite green on neem-4
- Benchmark: tensorized up to 4x faster, faster than rustworkx on 11 of 12 UCI datasets (Adult 156 vs 124 ms)

## Next
- Act on the user's answers to the two open questions

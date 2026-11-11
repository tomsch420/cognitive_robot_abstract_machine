## Plan
- Discuss the api-break (removed embedded_logsumexp, group_by_row, is_contiguous) with the probabilistic_model code owners before review

## Done
- reduce_by_row(values, ufunc) for every SumLayer reduction; entry_positions_by_rank documented with an example
- Removed embedded_logsumexp, group_by_row, is_contiguous; api-break label and migration notes
- Clean container suite green at 962bfa257c (1363 passed)

## Next
- Review

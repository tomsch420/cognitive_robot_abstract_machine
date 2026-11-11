## Plan
- Clean container suite run once the neem-4 sweep is done
- Discuss the api-break (removed embedded_logsumexp, group_by_row, is_contiguous) with the probabilistic_model code owners before review

## Done
- reduce_by_row(values, ufunc) for every SumLayer reduction; entry_positions_by_rank documented with an example
- Removed embedded_logsumexp, group_by_row, is_contiguous; api-break label and migration notes in the description

## Next
- Container suite, then report

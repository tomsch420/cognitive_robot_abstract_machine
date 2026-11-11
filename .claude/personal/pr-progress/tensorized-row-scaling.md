## Plan
- Ask the user: remove group_by_row and is_contiguous (no callers left)
- Clean container suite run once the neem-4 sweep is done

## Done
- reduce_by_row(values, ufunc) replaces logsumexp_by_row; SumLayer log and linear sums use it
- EntrySlot replaced by entry_positions_by_rank with a docstring example (review comment answered, resolved)
- embedded_logsumexp and its tests removed; CDF 5 to 22 % faster locally

## Next
- Act on the answer about group_by_row

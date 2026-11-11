## Plan
- Tests: log weighted sum of sum layers with uneven entry counts, all -inf children and one node equals a per node computation
- SumLayer streams np.logaddexp over entry slots, each slot over the prefix of nodes that have it (nodes ordered by entry count, fixed at construction)
- Benchmark before and after with evaluate.py on neem-4

## Done
- Profiled UCI Adult: 58 % embedded_logsumexp, 12 % group_by_row in sum layers; input layers cheap
- Prototype streaming over all slots: identical results, 1.3 to 2x faster

## Next
- Write the tests, then the streamed reduction

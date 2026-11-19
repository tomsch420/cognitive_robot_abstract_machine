PR tomsch420#51 (draft) - leaf-variables-per-node, plan folded-layered-circuits. Leaf layers store node_variables; mixed layers vectorize likelihood and CDF, run other queries per variable group.

Plan
- Done in 155782653d; next items folded-edge-matrix (parallel) and per-node-scopes (partial marginals, inner layer scopes).

Done
- 155782653d: node_variables, groups_of_one_variable, SumLayer.concatenation_of_pieces, union of discrete states, SymbolicLayer without domain_hashes.
- 1515 probabilistic_model tests, 7 circuit_learning and 10 probabilistic_model_experiments tests pass on neem-4 (worktree /root/folding).

Next
- Partial marginals of mixed leaf layers raise PartialMarginalOfLeafLayerError until per-node-scopes.

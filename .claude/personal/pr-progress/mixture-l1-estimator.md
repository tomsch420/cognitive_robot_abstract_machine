PR #769 (draft, bug): https://github.com/cram2/cognitive_robot_abstract_machine/pull/769
Supersedes #772 (closed, folded in).

Plan: total variation distance in [0, 1] for MonteCarloEstimator (mixture
importance sampling) and ShallowProbabilisticCircuit (exact, uniform leaves),
plus the shallow circuit bugs found on the way.

Done:
- MonteCarloEstimator.total_variation_distance (mean of |p-q|/(p+q) over samples
  from both models); removed l1_metric, its tolerance, and the uniform measure
  variant. Sources cited via :cite:p: (veach1995optimally, elvira2019generalized).
- ShallowProbabilisticCircuit.l1 -> total_variation_distance; subgraph_of fixed
  via mount; event_of_higher_density over cells_of_constant_density (midpoint
  bug, 1/2U[0,2)+1/2U[1,3) vs U[0,3) = 1/6).
- from_probabilistic_circuit rewritten via Unit.weighted_leaf_products; removed
  shallowing and remove_node_and_successor_structure.
- Tests: test_monte_carlo_estimator.py, test_rx/test_shallow_probabilistic_circuit.py,
  test_rx/test_shallow_circuit_conversion.py. Review comment (bibtex) resolved.

Next:
- docformatter missing locally; run scripts/format_docstrings.py once installed.
- Wait for CI and review; handle only when asked.

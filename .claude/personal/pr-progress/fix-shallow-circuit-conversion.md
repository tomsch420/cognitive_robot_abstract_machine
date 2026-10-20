PR #772 (draft, bug): https://github.com/cram2/cognitive_robot_abstract_machine/pull/772

Plan: make ShallowProbabilisticCircuit.from_probabilistic_circuit work on rx.

Done:
- Failing tests in test_rx/test_shallow_circuit_conversion.py (5).
- Unit.weighted_leaf_products (leaf/sum/product) + WeightedLeafProduct dataclass;
  from_probabilistic_circuit builds root sum of products from it; removed
  shallowing and remove_node_and_successor_structure.
- Pushed, draft PR opened with bug label.

Next:
- Wait for CI/review; handle only when asked.
- After it lands, PR #769's shallow circuit tests could use the conversion.

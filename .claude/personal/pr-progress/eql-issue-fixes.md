PR cram2#975 (draft) - fix the 11 EQL issues #928 #929 #937 #943 #944 #948 #949 #952 #964 #970 #971 in one branch.

Plan
- Get CI green and review; mark ready only when told.

Done
- All 11 fixed with failing-first tests; krrood EQL/RDR suites pass locally (1823).
- test_rule_tree_with_an_alternative now compares sets, test_nested_rule_explanation expects values (both approved).
- Pushed 8dc1e3e750, draft PR opened with bug-fix + api-break.
- CI on 8dc1e3e750: sdt test_explain_inferred_semantic_annotations expected the old explanation string -> updated (approved) in b9852d3562; giskardpy test_force_impact_node hit its tick limit (ROS timing, unrelated path).

Next
- Read CI on b9852d3562: sdt test (not runnable locally, casadi mismatch) and whether test_force_impact_node passes again.
- Discuss the api-break (abstract Call, alternative order and semantics, explanation format) with the KRROOD code owners.

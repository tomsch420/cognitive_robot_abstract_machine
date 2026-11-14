PR cram2#975 (draft) - fix the 11 EQL issues #928 #929 #937 #943 #944 #948 #949 #952 #964 #970 #971 in one branch.

Plan
- Get CI green and review; mark ready only when told.

Done
- All 11 fixed with failing-first tests; krrood EQL/RDR suites pass locally (1823).
- test_rule_tree_with_an_alternative now compares sets, test_nested_rule_explanation expects values (both approved).
- Pushed 8dc1e3e750, draft PR opened with bug-fix + api-break.

Next
- Discuss the api-break (abstract Call, alternative order and semantics, explanation format) with the KRROOD code owners.
- Read the CI result of 8dc1e3e750.

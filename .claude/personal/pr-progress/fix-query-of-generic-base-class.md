<!-- BEGIN-PR-PROGRESS -->
PR cram2/cognitive_robot_abstract_machine#1064 (draft), branch `fix-query-of-generic-base-class`.

Plan
- The EQL translator resolves queried types with `get_mapping_data_access_object_class`, so that a bare generic base selects from its own table.

Done
- Failing test with a generic base mimic, then the fix; tests pass on neem-4.
- The bullet world benchmark check: `MobileBase` is correct; all 2,028 `instances_of_class` queries are right.

Next
- Wait for CI and review.
- Still open, separately: `set_of` over open classes fails (33 queries).
<!-- END-PR-PROGRESS -->

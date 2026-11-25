<!-- BEGIN-PR-PROGRESS -->
PR cram2/cognitive_robot_abstract_machine#1065 (draft), branch `ormatic-eql-result-loading`.

Plan
- `ResultLoading` configures how `eql_to_sql` loads the selected objects; `ResultLoading.from_query` is the default.
- `psycopg[binary]` in semantic_digital_twin.

Done
- Loadings: lazy relationships, one query per class, selection join, columns, ids, pausing the garbage collector. Loading by hierarchy was dropped (SQLite's 64 table limit).
- Benchmarks on neem-4 (results/run3 to run5) and a garbage collector measurement are summarized in the PR description.
- Tests pass on neem-4; `krrood check` and `krrood format` are clean.

Next
- The user decides whether `from_query` should pause the garbage collector by default.
- After #1064 merges: `from_query` should use `get_mapping_data_access_object_class`.
- Discuss the api-break (lazy relationships by default) with the krrood and semDT code owners.
<!-- END-PR-PROGRESS -->

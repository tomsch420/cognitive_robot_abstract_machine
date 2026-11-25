<!-- BEGIN-PR-PROGRESS -->
PR cram2/cognitive_robot_abstract_machine#1065 (draft), branch `ormatic-eql-result-loading`. It also carries the generic base query fix; #1064 is closed in its favour.

Plan
- `ResultLoading` configures how `eql_to_sql` loads the selected objects; `ResultLoading.from_query` is the default.
- Queried types resolve through `get_mapping_data_access_object_class`, so that a bare generic base (`MobileBase`) selects from its own table.
- `psycopg[binary]` in semantic_digital_twin.

Done
- Loadings: lazy relationships, one query per class, selection join, columns, ids, pausing the garbage collector. Loading by hierarchy was dropped (SQLite's 64 table limit).
- The generic base fix, with a failing mimic test first; `from_query` uses the new lookup.
- Benchmarks on neem-4 (results/run3 to run5, check_g) and a garbage collector measurement are in the PR description.
- Tests pass on neem-4 (419 krrood, 77 SDT ORM); `krrood check` and `krrood format` are clean.

Next
- The user decides whether `from_query` should pause the garbage collector by default.
- Discuss the api-break (lazy relationships by default) with the krrood and semDT code owners.
- Still open, separately: `set_of` over open classes fails (33 queries).
<!-- END-PR-PROGRESS -->

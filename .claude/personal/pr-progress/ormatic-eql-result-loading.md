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
- Review threads answered and resolved at 53e4fdd78f: every option stays and its docstring says when to use it; `StoredObjectRow.objects_loaded_by_class` shares the loading by class; `object_daos` batches its ids.
- At 53e4fdd78f on neem-4: 336 krrood ORM, 77 SDT ORM and 31 EQL backend tests pass; `krrood check` is clean, also merged with main.

Next
- The user decides whether `from_query` should pause the garbage collector by default.
- Discuss the api-break (lazy relationships by default) with the krrood and semDT code owners.
- Still open, separately: `set_of` over open classes fails (33 queries).
<!-- END-PR-PROGRESS -->

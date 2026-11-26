PR cram2/cognitive_robot_abstract_machine#1065 (draft), branch `ormatic-eql-result-loading`. It also carries the generic base query fix; #1064 is closed in its favour.

Plan
- `ResultLoading` configures how `eql_to_sql` loads the selected objects; `ResultLoading.from_query` is the default.
- Queried types resolve through `get_mapping_data_access_object_class`; a class binding a parametrization the class diagram holds inherits it.
- `psycopg[binary]` in semantic_digital_twin; `krrood-ormatic` requires Python 3.12.

Done
- Loadings, benchmarks and the garbage collector measurement are in the PR description; review threads answered and resolved.
- xhigh code review fixed (backend, generics, distinct order on PostgreSQL, limited selection join, ON_ACCESS reach, rule breaks).
- `RelationshipLoading.FOR_CONVERSION` and the storage of a subclass in a field typed with a parametrization, at 1f3c44f1ff.
- Second review round at 4385ef1bbe: `GarbageCollectionPause` in the symbol graph, `inspect.isclass`, spelled out names; 393 krrood ORM, backend and symbol graph and 103 class diagram tests pass, merged `krrood check` clean.
- At 4dc4f4e6bb/1f3c44f1ff on neem-4 with regenerated ORMs: 391 krrood ORM and backend, 103 class diagram and 77 SDT ORM tests pass; `krrood check` clean on the merge with main 86cb692971.

Next
- The user decides whether `from_query` should pause the garbage collector by default.
- Discuss the api-break (lazy relationships by default, Python 3.12, ORM schema of classes binding a parametrization) with the krrood and semDT code owners.
- Still open, separately: `set_of` over open classes fails (33 queries); giskardpy's ORM tests were not run against the new schema.

<!-- BEGIN-PR-PROGRESS -->
PR cram2/cognitive_robot_abstract_machine#1065 (draft), branch `ormatic-eql-result-loading`. It also carries the generic base query fix; #1064 is closed in its favour.

Plan
- `ResultLoading` configures how `eql_to_sql` loads the selected objects; `ResultLoading.from_query` is the default.
- Queried types resolve through `get_mapping_data_access_object_class`, so that a bare generic base (`MobileBase`) selects from its own table.
- `psycopg[binary]` in semantic_digital_twin; `krrood-ormatic` requires Python 3.12.

Done
- Loadings, benchmarks and the garbage collector measurement are in the PR description.
- Review threads answered and resolved; every option documents when to use it.
- xhigh code review fixed at a8a7eb75db: backend loads as mapped, parametrized generic field, distinct order on PostgreSQL (checked on neem-4), limited selection join, type parameter field via subclass tables, ON_ACCESS reaching subclasses, renames, one batch size, column enum, Sphinx roles.
- At a8a7eb75db on neem-4: 373 krrood ORM and backend tests and 77 SDT ORM tests pass; `krrood check` clean, also merged with main 86cb692971.

Next
- The user decides whether `from_query` should pause the garbage collector by default, and whether a conversion profile (`selectin_loading`) becomes a `ResultLoading` option.
- Discuss the api-break (lazy relationships by default, Python 3.12) with the krrood and semDT code owners.
- Still open, separately: `set_of` over open classes fails (33 queries); storing a subclass of a generic in a field typed with a parametrization of it fails; the local import in `get_mapping_data_access_object_class` stays as asked.
<!-- END-PR-PROGRESS -->

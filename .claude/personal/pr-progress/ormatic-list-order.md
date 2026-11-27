PR #1077 (ormatic-list-order): index columns for ordered, nested and keyed collections.

Plan
- Address review comments as they come in.

Done
- 0e53e4b5a9: static container mapping, `_common_type`, tables_by_class reuse.
- 5e386f0d79: AnyCollection = Union, CollectionEntry, CollectionField + CollectionShape.fields_with_containers_to_restore, CollectionRelationship.is_innermost.
- bf7b607155: dao.py split into modules of at most 500 lines (user chose dependency inversion over local imports). PR description updated, all review threads replied and resolved.

Next
- Wait for review and CI. Locally unverified: experiments tests (their interface is not generated because coraplex generation fails on DetectAction, also before this PR), giskardpy tests.

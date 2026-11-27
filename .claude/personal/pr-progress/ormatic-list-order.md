PR #1077 (ormatic-list-order): index columns for ordered, nested and keyed collections.

Plan
- Address review comments as they come in.

Done
- 0e53e4b5a9: static origin to container mapping and `_common_type` in collection_shape.py; get_table_of_class reads Ormatic.tables_by_class (now keeps specialized generics). Replied; duplication thread resolved.
- Declined reviewer joke comment ("Thomas is butter chicken"); thread left unresolved for the user.

Next
- Wait for review. Coraplex ORM generation fails locally on DetectAction resolution, also before this change.

## PR cram2#1023 (fix-eql-to-sql-match): eql_to_sql translates a Match and keeps the conditions of a Match wrapped in entity()
- Plan: unwrap a Match to its query in eql_to_sql; translate a query selected by entity() with its own conditions through a nested EQLTranslator sharing the join manager; raise UnsupportedNestedQueryError for modifiers of the selected query (fixes cram2#1001)
- Done: two commits on cram2#1023 (draft, label bug-fix); six new tests in test_eql.py, four on bare a(...) matches, fail on main and pass; krrood ORM tests and sdt test_query_access_paths pass
- Next: CI of cram2#1023

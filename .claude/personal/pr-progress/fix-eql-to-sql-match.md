## PR cram2#1023 (fix-eql-to-sql-match): eql_to_sql translates a Match and keeps the conditions of a Match wrapped in entity()
- Plan: unwrap a Match to its query in eql_to_sql; translate a query selected by entity() with its own conditions through a nested EQLTranslator sharing the join manager; raise UnsupportedNestedQueryError for modifiers of the selected query (fixes cram2#1001)
- Done: three commits on cram2#1023 (draft, label bug-fix), last 84872d27b4 types query as HasExpression per review; seven new tests in test_eql.py pass
- Next: developer decides on review thread about eql_to_sql returning a CTE (proposed EQLTranslator.to_common_table_expression, api-break; this PR or a separate one); CI of cram2#1023

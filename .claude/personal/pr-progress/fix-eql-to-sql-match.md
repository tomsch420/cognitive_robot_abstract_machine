## PR cram2#1023 (fix-eql-to-sql-match): eql_to_sql translates a Match and keeps the conditions of a Match wrapped in entity()
- Plan: unwrap a Match to its query in eql_to_sql; translate a query selected by entity() with its own conditions through a nested EQLTranslator sharing the join manager; raise UnsupportedNestedQueryError for modifiers of the selected query (fixes cram2#1001)
- Done: four commits on cram2#1023 (draft, labels bug-fix, api-break), last d85d84fe26 drops eql_to_sql's as_common_table_expression so it always returns a translator; both review threads answered and resolved
- Next: discuss the api-break with krrood code owners; CI of cram2#1023

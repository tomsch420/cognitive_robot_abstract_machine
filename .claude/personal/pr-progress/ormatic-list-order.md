Branch `ormatic-list-order`, stacked on PR cram2#1065 (`ormatic-eql-result-loading`); fixes cram2#1026.
Plan file: ~/.claude/plans/snug-hopping-cosmos-of-a143fab8-dc42-4477-b03e-a5b96e7394fd.md

Plan:
- CollectionShape (levels + leaf) in krrood/ormatic/collection_shape.py
- JSON for collections of builtins (dicts and nesting included); association table with index_/key_ columns otherwise
- order_by on every association relationship (local, part, external part)
- to_dao flatten / from_dao rebuild nested lists and dicts, empty-collection marker rows
- finalize nested containers and JSON dict keys by shape

Done:
- branch created from PR 1065 head 1f3c44f1ff

Next:
- mimics in test/krrood_test/dataset/ordered_collections.py + failing tests

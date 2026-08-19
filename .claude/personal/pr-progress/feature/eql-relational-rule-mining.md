# PR #26 — engine-atom-refinement (eql-relational-rule-mining, wave 1)

Plan approved via plan-item-kickoff; full plan recorded in
`.claude/personal/plans/eql-relational-rule-mining/roadmap.md` under
"`engine-atom-refinement` — implementation plan (approved)".

## Current design (post-review, see "Done" below)
New self-contained krrood subpackage `krrood/entity_query_language/rule_mining/`
(`__init__.py` empty) with `candidate_rule.py`'s `CandidateRuleBody` — a
**plain mutable dataclass** (`eq=False`, list fields), whose three operator
methods mutate in place and `return self`, matching `Query.where()`'s own
established convention (not `dataclasses.replace`, per review feedback):

- `extend_with_related_variable` (dangling atom) — inlines
  `DataclassOnlyIntrospector().discover(...)` directly for the attribute
  check (no separate `candidate_attribute_names` wrapper — removed per
  review), uses `get_field_type_endpoint` + `flat_variable` for collections.
- `constrain_variable_to_value` (instantiated atom).
- `close_by_equating_variables` (closing atom).
- `to_query()` — `entity(head_variable).where(*conditions)`.

Class docstring carries a doctest-style example (`>>>`, checked output),
matching `verbalizer.py`'s convention — verified via `doctest.testmod()`,
though it isn't wired into the automated verbalization doctest harness
(that only scans the `verbalization`/`testing` packages).

Fixture (`test/krrood_test/dataset/rule_mining_fixture.py`): `Handle`/
`Container`, both declared `eq=False` directly (matching `Symbol`'s own
default) rather than hand-written `__hash__`/`__eq__` — avoids recursing
across their back-reference cycle.

Tests: `test/krrood_test/test_eql/test_rule_mining/test_candidate_rule.py`
(7 tests, all passing).

## Done
- Branch + draft PR #26 created, plan.yaml item flipped to in_progress,
  plan recorded in roadmap.md.
- Implemented `CandidateRuleBody` + fixture + tests, full krrood suite
  green (2156 passed, 6 skipped — one pre-existing `mypy`-dependent
  module excluded, unrelated to this change).
- Addressed two rounds of PR review comments (all 7 threads resolved):
  reused `DataclassOnlyIntrospector` instead of hand-rolled field
  enumeration; emptied `__init__.py`; removed the now-unnecessary
  `candidate_attribute_names` wrapper; switched from frozen-dataclass +
  `replace` to mutable-dataclass + mutate-and-return-self; replaced
  "query graph" wording with "query" throughout; added the doctest
  example; dropped the fixture's redundant custom `__hash__`/`__eq__`.

## Next
- Item is functionally complete pending any further review. Nothing else
  planned for this item — `status` intentionally left `in_progress` in
  `plan.yaml`; the dashboard's automatic merged-PR correction handles
  flipping it to `done`.

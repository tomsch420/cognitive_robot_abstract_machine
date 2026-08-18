# PR #26 — engine-atom-refinement (eql-relational-rule-mining, wave 1)

Plan approved via plan-item-kickoff; full plan recorded in
`.claude/personal/plans/eql-relational-rule-mining/roadmap.md` under
"`engine-atom-refinement` — implementation plan (approved)".

## Plan summary
New self-contained krrood subpackage `krrood/entity_query_language/rule_mining/`
with a `CandidateRuleBody` dataclass (immutable-update) and three
refinement operators: `extend_with_related_variable` (dangling atom, via
`get_field_type_endpoint` + `flat_variable` for collections),
`constrain_variable_to_value` (instantiated atom),
`close_by_equating_variables` (closing atom). Plus a
`candidate_attribute_names` helper. TDD: new fixture
`test/krrood_test/dataset/rule_mining_fixture.py`, tests in
`test/krrood_test/test_eql/test_rule_mining/test_candidate_rule.py`.

## Done
- Branch + draft PR #26 created, plan.yaml item flipped to in_progress,
  plan recorded in roadmap.md.
- Fixture `test/krrood_test/dataset/rule_mining_fixture.py` (`Container`/
  `Handle`, scalar back-reference + collection field) and 8 TDD tests in
  `test/krrood_test/test_eql/test_rule_mining/test_candidate_rule.py`,
  written before the implementation.
- Implemented `CandidateRuleBody` (frozen, `eq=False` — comparing EQL
  variables with `==` builds a `Comparator`, not a bool, so the dataclass
  default `__eq__`/`in`/`==` would be misleading; equality/removal use
  identity checks instead) with `extend_with_related_variable`,
  `constrain_variable_to_value`, `close_by_equating_variables`, and
  `to_query`, plus `candidate_attribute_names` (excludes private/
  `Symbol`-bookkeeping fields such as `_inference_explanation_`) and
  `UnknownAttributeError`/`IncompatibleVariableTypesError` in
  `krrood/entity_query_language/rule_mining/`.
- All 8 new tests pass; full krrood suite green (2157 passed, 6 skipped,
  0 failed — the one pre-existing `mypy`-dependent typing test was
  excluded, since `mypy` isn't installed in this environment, unrelated
  to this change).
- Self-containment verified by import inspection: new files import only
  from `krrood` (plus stdlib/`typing_extensions`), nothing from
  `semantic_digital_twin` or other workspace packages.
- `scripts/format_docstrings.py` run on all new files.
- Commit `0f144e47e` pushed to `feature/eql-relational-rule-mining` /
  PR #26 (still draft). PR description already matched the implementation,
  no update needed.

## Next
- Item is functionally complete; `plan.yaml`'s `status` is left
  `in_progress` per instructions (only the dashboard's automatic
  merged-PR correction should flip it to `done`).
- Next plan item is `engine-support-confidence` (support/confidence
  scoring and pruning over candidate rule bodies), which depends on this
  one.
- Open design note carried into that item: `extend_with_related_variable`
  folds the new variable into `conditions` as a bare (non-comparator)
  expression, relying on EQL's truthiness handling of `.where()` conditions
  to create the join/flatten; `to_query()` still only ever selects
  `head_variable`, so "joined pairs" are recovered by dereferencing the
  attribute on each returned instance rather than via `set_of`. Worth a
  sanity check if the scoring/mining-loop item finds this awkward.

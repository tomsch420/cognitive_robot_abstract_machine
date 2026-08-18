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

## Next
- Write the fixture and the tests (TDD — tests before implementation).
- Implement `CandidateRuleBody` + the three operators + `candidate_attribute_names`.
- Run `scripts/format_docstrings.py` on new files.
- Verify: new tests pass, full krrood suite green, self-containment
  (no import outside krrood + random_events/probabilistic_model).

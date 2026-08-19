# PR #27 — engine-support-confidence (eql-relational-rule-mining, wave 1)

Plan approved via plan-item-kickoff; full plan recorded in
`.claude/personal/plans/eql-relational-rule-mining/roadmap.md` under
"`engine-support-confidence` — implementation plan (approved)".

Stacked on PR #26 (`feature/eql-relational-rule-mining`, still open/draft —
functionally complete). Branch reset from `main` onto that branch before
this item's work started.

## Design
New `krrood/entity_query_language/rule_mining/scoring.py`:
`ScoreThresholds` (`minimum_support`, `minimum_confidence`) and `RuleScore`
(`support`, `confidence`, `meets(thresholds)`) dataclasses.
`CandidateRuleBody.score() -> RuleScore` (new method next to `to_query()`):
support = `len(list(self.to_query().evaluate()))`; confidence =
support / count of `entity(head_variable).where(*conditions[:-1]).evaluate()`
(the body minus its last atom). `prior_support == 0` → confidence `0.0`.
Empty `conditions` → new `EmptyRuleBodyError`.

## Done
- Branch reset onto `feature/eql-relational-rule-mining`, bootstrap commit
  pushed, draft PR #27 created (base `feature/eql-relational-rule-mining`).
- `plan.yaml` item flipped to `in_progress`, plan recorded in `roadmap.md`.

## Next
- TDD: write `test/krrood_test/test_eql/test_rule_mining/test_scoring.py`
  first (support/confidence after one atom; after closing two prior atoms;
  `prior_support == 0` case; `EmptyRuleBodyError`; `RuleScore.meets`).
- Implement `EmptyRuleBodyError`, `ScoreThresholds`, `RuleScore`,
  `CandidateRuleBody.score()`.
- Run full krrood suite, `scripts/format_docstrings.py`, push, republish
  the plan dashboard.

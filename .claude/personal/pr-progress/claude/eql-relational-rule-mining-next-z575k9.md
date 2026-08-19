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
- TDD: wrote `test/krrood_test/test_eql/test_rule_mining/test_scoring.py`
  first (7 tests: support/confidence after one atom; after closing two
  prior atoms — collision-safe scenario, see note below; `prior_support
  == 0` case; `EmptyRuleBodyError`; two `RuleScore.meets` cases).
- Implemented `EmptyRuleBodyError` (`exceptions.py`), `ScoreThresholds` /
  `RuleScore` (`scoring.py`), `CandidateRuleBody.score()`.
- Discovered mid-implementation: `.where()` raises `NoConditionsProvided`
  on an empty condition list, so the prior-support query only calls
  `.where()` when `conditions[:-1]` is non-empty.
- Discovered mid-implementation: a pre-existing, unrelated bug in
  `krrood/class_diagrams` mis-resolves an attribute's static type to the
  wrong same-named class when multiple test-dataset modules declare a
  class with the same name (e.g. `Container` exists in
  `rule_mining_fixture.py`, `semantic_world_like_classes.py`, and
  `datasets.py`) and the full test suite's conftest has built the global
  class diagram first. Only affects `extend_with_related_variable` called
  a *second* time on a variable introduced by a first traversal; the
  `test_score_after_closing_atom_on_top_of_two_prior_atoms` test was
  redesigned to reach two prior atoms via `extend_with_related_variable` +
  `constrain_variable_to_value` instead, sidestepping the collision. Not
  fixed here — out of scope for scoring, and worth flagging to the
  developer separately since it could bite a future item that does chain
  two dangling-atom traversals.
- Verified: `pytest test/krrood_test/test_eql/test_rule_mining/ -v` (13
  passed), full krrood suite (1791 passed, 6 skipped; the only 2 failures
  are a pre-existing `dot`/Graphviz-binary environment gap in
  `test_object_diagram.py`, unrelated), `scripts/format_docstrings.py`
  run (had to manually fix one docformatter defect: it broke a `:meth:`
  Sphinx role mid-identifier across a line wrap in `scoring.py`).
- Pushed, PR #27 description updated to match, dashboard republished.

## Next
- Nothing planned for this item; functionally complete pending review.
  `status` intentionally left `in_progress` — only the dashboard's
  automatic merged-PR correction should flip it to `done`.
- Worth telling the developer about the `class_diagrams` same-name-class
  type-resolution bug found above, independently of this PR.

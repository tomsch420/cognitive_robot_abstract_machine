# PR #28 — engine-tests-synthetic (eql-relational-rule-mining, wave 1)

Plan approved via plan-item-kickoff; full plan recorded in
`.claude/personal/plans/eql-relational-rule-mining/roadmap.md` under
"`engine-tests-synthetic` — implementation plan (approved)".

Stacked on PR #27 (`claude/eql-relational-rule-mining-next-z575k9`, itself
stacked on PR #26 — both still open/draft). This item's own scope turned
out bigger than its title: items 1-2 only built the operators + scorer,
nothing before this actually searches, so this item builds the mining
search loop itself, not just a test for one.

## Design
New `krrood/entity_query_language/rule_mining/miner.py`: `RuleMiner`
(`thresholds: ScoreThresholds`, `maximum_atoms: int`), `.mine(head_type,
domain) -> List[CandidateRuleBody]`. Breadth-first up to `maximum_atoms`;
depth-0 frontier = empty body + one self-join-seeded body (second
independent same-typed variable, scoped to head's own type only); each
step tries every declared-attribute dangling extension (via
`DataclassOnlyIntrospector().discover(...)`, matching what
`extend_with_related_variable` already uses internally) plus
`close_by_equating_variables` on every open type-compatible pair; prunes
branches with `support < minimum_support`; collects bodies once every
variable the miner itself is tracking as needing closure has been closed
and both thresholds are met.

Key empirical finding driving that "miner tracks its own closure set"
design: `CandidateRuleBody.open_variables` only ever loses an entry via
`close_by_equating_variables` — a manually-seeded self-join variable used
only as a further traversal source never clears on its own. Verified by
running the actual merged code, not by reading the docstring alone.

Planted rule: reuses the existing `Container`/`Handle` fixture (no new
dataset file) — "two handles are siblings iff they share a container",
verified against the real engine: support=9, confidence=0.36.

## Done
- Branch `engine-tests-synthetic` created off
  `claude/eql-relational-rule-mining-next-z575k9`, bootstrap commit
  pushed, draft PR #28 created.
- `plan.yaml` item flipped to `in_progress`, plan recorded in `roadmap.md`.

## Next
- TDD: write `test/krrood_test/test_eql/test_rule_mining/test_miner.py`
  first (build the sibling-handle world, run `RuleMiner(...).mine(...)`,
  assert a returned body's evaluated result matches
  `['H1','H1','H2','H2','H3','H3','H4','H4','H5']` and its `score()`
  equals `RuleScore(support=9, confidence=0.36)`).
- Implement `RuleMiner` in `miner.py`.
- Run full krrood suite, `scripts/format_docstrings.py`, push, republish
  the plan dashboard.
- After this item: continue autonomously through the plan per the user's
  "finish it all!" — next up is `partnet-remote-access`, which needs SSH
  access to neem-4.informatik.uni-bremen.de this session doesn't have;
  flag that clearly rather than faking it when reached.

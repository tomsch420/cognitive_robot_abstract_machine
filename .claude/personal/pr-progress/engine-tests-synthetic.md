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
- TDD: wrote `test/krrood_test/test_eql/test_rule_mining/test_miner.py`
  first (builds the sibling-handle world, runs `RuleMiner(...).mine(...)`,
  asserts a returned body's evaluated result matches
  `['H1','H1','H2','H2','H3','H3','H4','H4','H5']` and its `score()`
  equals `RuleScore(support=9, confidence=0.36)`).
- Implemented `RuleMiner` in `miner.py`, plan-replay based (see Design).
- Discovered mid-implementation, empirically (not assumed):
  `copy.deepcopy` on a `CandidateRuleBody` recurses indefinitely — the EQL
  variable objects' dynamic `__getattr__`/mapped-attribute machinery
  breaks `copy`'s dict-key hashing during traversal. Redesigned branching
  around replayable plans (`SelfJoinSeedStep`/`ExtendStep`/`CloseStep`,
  addressed by plan-step index) instead of copying live bodies, before
  writing any of the actual search loop.
- Landed on the final "is_closed" definition (every introduced variable is
  either closed or used as an extend-source) only after the first
  definition (every introduced variable individually closed) proved
  unsatisfiable for the self-join pattern — the seed variable itself is
  never directly closed, only used as a traversal source. Verified the
  final definition finds the planted rule before finalizing it.
- Verified `Handle` also collides by name across 3 test-dataset modules
  (same shape as the `Container` collision found in #27), which is why the
  search deliberately never chains a second dangling-atom hop — confirmed
  by testing the full search end-to-end and getting a clean pass on the
  first real run, no crash from that bug.
- Verified: `pytest test/krrood_test/test_eql/test_rule_mining/ -v` (14
  passed), full krrood suite (1792 passed, 6 skipped, same 2 pre-existing
  `dot`/Graphviz-binary failures as #27, unrelated),
  `scripts/format_docstrings.py` run clean (no manual fixes needed this
  time).
- Pushed, PR #28 description updated to match, dashboard republished.

## Next
- Nothing planned for this item; functionally complete pending review.
  `status` intentionally left `in_progress` — only the dashboard's
  automatic merged-PR correction should flip it to `done`.
- Continuing autonomously through the plan per the user's "finish it
  all!" — next up is `partnet-remote-access`, which needs SSH access to
  neem-4.informatik.uni-bremen.de this session doesn't have; flag that
  clearly rather than faking it when reached.

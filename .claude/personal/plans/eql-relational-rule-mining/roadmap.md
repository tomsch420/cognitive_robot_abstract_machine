# Relational rule mining with EQL — roadmap

## Why

`semantic_digital_twin`'s PartNet-Mobility loader
(`semantic_digital_twin/src/semantic_digital_twin/adapters/partnet_mobility_dataset/loader.py`,
`_apply_semantics_to_world`) currently converts PartNet-Mobility's own flat
per-part labels (`semantics.txt`) into semantic annotations by a 1:1 rename —
`part_net_semantic_annotations_class(root=body)` — into a generated stub class
with no relational structure at all. The real, richer annotation model
already exists in the same package
(`semantic_digital_twin/semantic_annotations/semantic_annotations.py`):
`Drawer(Furniture, HasCaseAsRootBody, HasHandle, HasMechanicalJoint)`,
`Cabinet(Furniture, HasCaseAsRootBody, HasDoors, HasDrawers)`,
`Door(HasHandle, HasMechanicalJoint)`, with real relational fields
(`HasHandle.handle`, `HasMechanicalJoint.mechanical_joint`,
`HasDrawers.drawers`, `HasDoors.doors` in `mixins.py`). Nothing currently
populates those relational fields automatically from raw data.

This plan's goal: build a tool, driven by data rather than hand-written
rules, that mines the relational pattern connecting a body's flat label +
joint type + position in the kinematic tree to the *structured* annotation
it should become — then use that to actually generate real
`Drawer`/`Cabinet`/`Door` annotations for PartNet-Mobility models.

## How the approach was chosen

A survey of the state of the art (classical ILP/TILDE, modern ILP/Popper,
AMIE-family knowledge-graph rule mining, differentiable/neuro-symbolic
learners, statistical relational learning) concluded:

- **AMIE-family closed-world association-rule mining** needs no labeled
  examples — it mines directly over whatever relational facts already
  exist, scoring candidate rule bodies by support/confidence. Its
  refinement operators (add a dangling atom / an instantiated atom / close
  the rule) map directly onto incrementally building an EQL
  `entity(...).where(...)` predicate — the query graph *is* the rule, and
  `query.evaluate()` *is* the scorer. This became the wave 1/2 anchor.
- **TILDE** (relational decision trees, ~1998) is a supervised
  predecessor in the same family as **Popper** (2021+, "learning from
  failures" via ASP) — Popper is its modern, more efficient successor,
  producing exact provably-covering Horn clauses rather than a tree. Both
  need labeled positive/negative examples, which this domain doesn't have
  up front. This becomes the wave 3 supervised refinement step, using
  cases RDR has confirmed as the label source.
- **Differentiable/neuro-symbolic** (Neural LP, DRUM, RNNLogic) and
  **statistical relational learning** (MLN, ProbLog) were ruled out: both
  trade away the exact, readable, Python-native predicates that are EQL's
  core design bet.

krrood's object model is not a classical flat RDF triple store — entities
are typed dataclasses with n-ary structure, nested/collection-valued
attributes, and a real Python class hierarchy instead of type-triples.
The core method family (relational association-rule mining, the
WARMR→AMIE lineage) is schema-agnostic and transfers regardless; AMIE's
specific triple-store optimizations (PCA confidence, dangling-atom
enumeration tuned for RDF scale) are inspiration, not a literal target to
port. krrood's static typing is, if anything, an advantage over a raw KG:
candidate-atom generation can be pruned by declared field types instead of
needing separate type-inference heuristics, and mining is naturally scoped
per Python type via `variable(SomeType, domain=...)`.

**Real risk to design around, not gloss over:** AMIE-style statistical
confidence is calibrated for KGs with millions of facts. krrood/SemDT
domains are far smaller, so percentage-confidence pruning alone would be
noisy. The engine (wave 1) uses absolute-count thresholds alongside
percentage confidence for exactly this reason, and wave 1's validation
uses a planted-rule synthetic fixture (exact recall, not statistical
plausibility) before ever touching real data.

## Why PartNet-Mobility, and why this dataset in particular

Two earlier candidate use cases were considered and superseded:

1. **A purely synthetic toy world** — rejected; the user wanted something
   real and data-driven, not invented for this plan.
2. **krrood's existing handles-and-containers RDR test fixture**
   (`test/krrood_test/test_ripple_down_rules/`) — a real fixture already
   in the repo, with a hand-written ground-truth rule
   (`test_expert_answers/drawer_cabinet_expert_answers_fit.json`:
   `Drawer(handle, container)` iff a `FixedConnection` joins them,
   `Cabinet` iff a `PrismaticConnection` joins a container to a drawer's
   container) and labeled positive/negative examples
   (`get_possible_drawers()`'s full handle×container cross-product). This
   remains an excellent *validation* target for the mining engine (wave 1
   sanity-checking) precisely because a human already wrote the rule by
   hand for comparison — but it's too small (2 handles × 2 containers) to
   be the plan's primary target.
3. **PartNet-Mobility** (chosen): a real, large public dataset (2218
   articulated object models, 345 in the `StorageFurniture` category
   alone) with exactly the same relational shape (parts + joint types +
   kinematic tree) at real scale, and a genuine downstream deliverable —
   replacing `_apply_semantics_to_world`'s flat label copy with real
   structural annotations.

The dataset is already downloaded in full on
`neem-4.informatik.uni-bremen.de`, at
`/raid/users/tom_sch/datasets/partnet-mobility-dataset` (2218 models,
8.1GB; category counts include 345 `StorageFurniture`, 95 `Table`, 84
`Faucet`, ... — see `meta.json`'s `model_cat` field per model), alongside
`/raid/users/tom_sch/datasets/partnet_meta`. A `SAPIEN_ACCESS_TOKEN` is
also available locally (in the user's `.bashrc`) as a fallback for the
existing token-gated loader path, but the plan does not depend on it: the
corpus is used directly from neem-4, not re-downloaded.

**Deliberate choice: no local copy, no git-committed dataset fixture.**
This repo's existing convention for large external test datasets
(`test/semantic_digital_twin_test/test_sage_10k/test_loader.py`) is to
fetch live with a skip-guard rather than commit raw data to git. Copying
even the 345-model `StorageFurniture` subset into a git-tracked directory
would meaningfully bloat the repo. Instead: local development and CI use
small synthetic EQL fixtures (wave 1); real corpus runs are invoked over
SSH directly on neem-4 against the existing `/raid` path (wave 2+); no
new dataset fixture is committed to git, and CI-facing tests keep the
existing `SAPIEN_ACCESS_TOKEN`-gated `skipif` pattern from
`test_partnet_mobility.py`.

## Why the engine/bridge package split

`AGENTS.md` requires krrood to stay self-contained: it may never import
from `semantic_digital_twin` (the only permitted exceptions are
`random_events` and `probabilistic_model`). Since the PartNet-Mobility
bridge necessarily instantiates `semantic_digital_twin`'s own
`Drawer`/`Cabinet`/`Door`/`HasHandle`/`HasMechanicalJoint` classes, that
half of the work cannot live inside krrood.

The resolution: split into a domain-agnostic mining **engine** (new krrood
subpackage, e.g. `krrood/entity_query_language/rule_mining` — knows
nothing about drawers, cabinets, or PartNet-Mobility, just EQL queries as
rule hypotheses) and a domain-specific **bridge** (lives in
`semantic_digital_twin`, depends on krrood's engine plus
`semantic_digital_twin`'s own classes). Waves 1 vs. 2/3 in `plan.yaml`
mirror this split directly.

## RDR as the human-in-the-loop half

krrood already has a Ripple Down Rules system
(`krrood/ripple_down_rules`, `krrood/entity_query_language/rdr`) that is
expert-driven and incremental but has no automatic rule-proposal
mechanism. Mining is designed to be the automatic "propose" half of a
propose/correct loop: mined rules (with confidence scores) become
candidate conclusions fed into RDR's `fit_case` flow, for a human to
accept, reject, or refine as an exception (wave 2's
`partnet-rdr-bridge` item). Cases RDR confirms this way then become the
label source for wave 3's supervised ILP pass — closing the loop between
the unsupervised and supervised halves of the plan.

## Open decision, deferred rather than assumed

Whether the finished pipeline *replaces*
`_apply_semantics_to_world`'s flat label copy, or runs alongside it as an
opt-in richer-annotation path, is left to wave 3's evaluation item
(`evaluate-against-ground-truth`) rather than decided up front.

## `engine-atom-refinement` — implementation plan (approved)

New subpackage: `krrood/entity_query_language/rule_mining/`
(`__init__.py`, `candidate_rule.py`, `exceptions.py`) — self-contained,
no import outside krrood + random_events/probabilistic_model.

**`CandidateRuleBody`** (dataclass, `candidate_rule.py`): the rule body
under construction — `head_variable`, `open_variables` (dangling, not yet
connected to another variable in the body), `conditions` (accumulated
`.where()` conditions). Three immutable-update extension methods, each
returning a new `CandidateRuleBody`:

- `extend_with_related_variable(source_variable, attribute_name)` — the
  *dangling-atom* operator. Resolves the attribute's static type via
  `get_field_type_endpoint` (`krrood/symbol_graph/helpers.py`, the same
  helper `Attribute._update_type_` already uses in
  `core/mapped_variable.py`); flattens collection-valued attributes via
  `flat_variable()`. Raises `UnknownAttributeError` for an attribute not
  declared on the type.
- `constrain_variable_to_value(variable, value)` — the *instantiated-atom*
  operator: adds `variable == value` to `conditions`.
- `close_by_equating_variables(variable_a, variable_b)` — the
  *closing-atom* operator: adds `variable_a == variable_b`, removes both
  from `open_variables`. Raises `IncompatibleVariableTypesError` if the
  two variables' static types share no common subtype.
- `to_query()` — `entity(self.head_variable).where(*self.conditions)`; the
  query graph *is* the rule, no separate rule representation.

Plus `candidate_attribute_names(type_)` — lists a dataclass type's
declared field names, for later use by item 3's mining loop (not wired
into a search loop by this item).

**Naming note**: deliberately avoids the word "Refinement" —
`krrood.entity_query_language.rules.conclusion_selector.Refinement`
already names an unrelated concept (an RDR tree branch).

**Tests first** (TDD): new fixture
`test/krrood_test/dataset/rule_mining_fixture.py` (a `Container` with a
`handles: List[Handle]` collection field, `Handle.container: Container`
scalar back-reference — following `department_and_employee.py`'s style,
adding a collection-valued field to exercise flattening). New tests under
`test/krrood_test/test_eql/test_rule_mining/test_candidate_rule.py`
covering: scalar and collection-valued `extend_with_related_variable`,
the unknown-attribute exception, `constrain_variable_to_value`,
`close_by_equating_variables` (success and incompatible-types exception),
`candidate_attribute_names`, and the immutable-update property (original
body unaffected by an extension). All assertions against concrete
expected values, not not-None/not-empty checks.

**Verification**: `pytest test/krrood_test/test_eql/test_rule_mining/ -v`,
full krrood suite still green, self-containment verified by import
inspection, `scripts/format_docstrings.py` run on new files.

**Open items flagged at approval time, not yet resolved**:
- The three method names above are this session's proposal (roadmap/item
  notes named the operator *kinds*, not exact method names) — worth a
  sanity check if a later session finds them awkward to use.
- No explicit backtracking/undo method on `CandidateRuleBody` — the
  immutable-update design means an earlier reference already serves that
  purpose; flagged in case item 3's mining loop finds it insufficient.

## `engine-support-confidence` — implementation plan (approved)

Depends on `engine-atom-refinement`'s `CandidateRuleBody` (PR #26, feature
branch `feature/eql-relational-rule-mining` — functionally complete, still
open/draft). This item stacks directly on that branch rather than waiting
for it to merge, per this repo's normal stacked-PR workflow;
`check_dependency_readiness.py` correctly flags it as not yet `is_ready`
(a draft PR hasn't had its author review), noted here rather than silently
proceeding as if it were.

New file `krrood/entity_query_language/rule_mining/scoring.py`:

```python
@dataclass
class ScoreThresholds:
    minimum_support: int
    minimum_confidence: float

@dataclass
class RuleScore:
    support: int
    confidence: float

    def meets(self, thresholds: ScoreThresholds) -> bool: ...
```

`CandidateRuleBody.score() -> RuleScore` (new method in `candidate_rule.py`,
next to `to_query()` — scoring is the body's own behaviour):

- **Support** = `len(list(self.to_query().evaluate()))` — the count of
  bindings satisfying the full body, per `plan.yaml`'s note ("evaluating
  the candidate rule-body EQL query itself... no separate inference
  engine"). Each row already reflects one join binding, so no
  `set_of(head_variable, *open_variables)` multi-select is needed for a
  plain count — this resolves `engine-atom-refinement`'s open note about
  recovering joined pairs from `to_query()` results: recovery is only
  needed if a *value*, not just a count, is wanted, which is out of this
  item's scope.
- **Confidence** = `support / prior_support`, where `prior_support` is
  `entity(self.head_variable).where(*self.conditions[:-1]).evaluate()`'s
  count — the body with its most-recently-added atom removed, reconstructed
  directly from `self.conditions` rather than a stored "previous body"
  reference. This also resolves `engine-atom-refinement`'s other open
  note (no undo/backtracking method on the now-mutating `CandidateRuleBody`)
  for this use: `conditions[:-1]` doesn't need one.
- `prior_support == 0` → confidence `0.0` (not an error: both counts being
  0 is a normal empty-candidate outcome during search, filtered out by
  `minimum_support` later).
- `self.conditions` empty when `score()` is called → raises new
  `EmptyRuleBodyError` (`exceptions.py`, following the existing
  `DataclassException` convention) — a genuine precondition violation, not
  a search-time edge case.

Tests first (TDD): `test/krrood_test/test_eql/test_rule_mining/test_scoring.py`,
reusing `containers_and_handles()` from `test_candidate_rule.py` via a
relative import. Cases: support/confidence after one
`extend_with_related_variable`; after a `close_by_equating_variables` on
top of two prior atoms (exercises `conditions[:-1]` against more than one
prior atom); the `prior_support == 0` → confidence `0.0` case;
`EmptyRuleBodyError` on a fresh body; `RuleScore.meets(thresholds)` (both
clearing thresholds, and either one not).

Verification: `pytest test/krrood_test/test_eql/test_rule_mining/ -v`, full
krrood suite green, self-containment verified, `scripts/format_docstrings.py`
run on new/changed files.

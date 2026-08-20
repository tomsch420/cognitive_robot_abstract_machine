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

## `engine-tests-synthetic` — implementation plan (approved)

Items 1–2 (`engine-atom-refinement` PR #26, `engine-support-confidence`
PR #27, both still open/draft) built only the pieces: `CandidateRuleBody`'s
three refinement operators and `score()`. Neither built anything that
actually searches — nothing enumerates candidate atoms, extends bodies,
prunes by score, or decides when a body is a finished rule. This item's own
note ("the miner must recover [a planted rule]") assumes a miner exists;
it doesn't, so this item builds the search loop itself, not just a test
for one.

**Design gap found empirically, and how it's resolved**:
`CandidateRuleBody.open_variables`'s docstring says it tracks variables
"not yet connected... by a closing atom", but this is narrower in the
merged code than it sounds: only `close_by_equating_variables` ever
removes an entry. Using a variable as the source of a further
`extend_with_related_variable` hop does not remove it, and a
manually-seeded second variable (the pattern
`test_close_by_equating_variables_...` already uses) only clears once
*it itself* is passed to a closing call. Verified directly:

```python
head = variable(Handle, domain=handles)
other = variable(Handle, domain=handles)
body = CandidateRuleBody(head_variable=head, open_variables=[other])
body.extend_with_related_variable(head, "container")
body.extend_with_related_variable(other, "container")
body.close_by_equating_variables(<container-of-head>, <container-of-other>)
# open_variables ends as [Handle]  <- `other` never clears, permanently "open"
```

So `open_variables` alone can't tell a miner "is this body a finished,
closed rule" whenever a self-join-style auxiliary variable is involved.
Rather than touch `CandidateRuleBody` (out of scope; still mid-review on
an open PR), the miner tracks its own "still needs a closing partner" set,
seeded from every variable it itself introduces, independent of
`CandidateRuleBody.open_variables` (which is still used for what it's
clearly built for: the frontier of variables available to extend or close
from).

**The planted rule**: reuses the existing
`test/krrood_test/dataset/rule_mining_fixture.py` (`Container`/`Handle`) —
no new fixture file. This also sidesteps a pre-existing, unrelated bug
found while building item 2 (a same-named-class type-resolution collision
in `krrood/class_diagrams`, triggered by a *second*
`extend_with_related_variable` hop off a variable introduced by a first
one): this rule only ever does one hop per variable.

World: `c1` holds `{h1, h2}`, `c2` holds `{h3, h4}`, `c3` holds `{h5}`
alone. Rule: two `Handle`s are "siblings" iff they share a container — a
self-join pattern (AMIE's third refinement family, not one of items 1–2's
three operators, but expressible today purely as miner-side search: seed
a second, independently-domained `Handle` variable at depth 0, then let
ordinary dangling + closing atoms take it from there). Verified against
the real engine (not hand math): support=9, confidence=0.36, result set
`['H1','H1','H2','H2','H3','H3','H4','H4','H5']`.

**New file** `krrood/entity_query_language/rule_mining/miner.py`:

```python
@dataclass
class RuleMiner:
    thresholds: ScoreThresholds
    maximum_atoms: int

    def mine(self, head_type: type, domain: Sequence) -> List[CandidateRuleBody]: ...
```

Breadth-first, level by level, up to `maximum_atoms`:
- Frontier seeding (depth 0): the empty body over `head_type`/`domain`,
  plus one self-join-seeded body — self-join seeding is scoped to only the
  head's own type, at depth 0 only, a deliberate wave-1 limit called out
  rather than generalized now.
- Refinement per body: for every variable the miner has introduced so far,
  `deepcopy` the body (operators mutate in place) and try every declared
  attribute name (via `DataclassOnlyIntrospector().discover(...)`, the
  same introspector `extend_with_related_variable` already uses
  internally — no probing via exceptions, per AGENTS.md) as a dangling-atom
  extension; also try `close_by_equating_variables` on every pair of
  currently-open, type-compatible variables.
- Scoring/pruning: skip scoring a body with empty `conditions`; otherwise
  drop the whole branch if `support < thresholds.minimum_support` (support
  is monotonically non-increasing as atoms are added — standard, safe
  AMIE pruning).
- Collecting results: a body counts as a finished rule once every variable
  the miner is tracking as needing closure has been closed and its score
  meets both thresholds.
- Deliberately not deduplicating semantically-equivalent rules reached via
  different atom orders — out of scope for a wave-1 validation pass.

Intentionally the minimum needed to recover the one planted rule — not the
production miner wave 2 will run against PartNet-Mobility.

Tests first (TDD): `test/krrood_test/test_eql/test_rule_mining/test_miner.py`
builds the world above, runs `RuleMiner(ScoreThresholds(minimum_support=2,
minimum_confidence=0.1), maximum_atoms=3).mine(Handle, [h1..h5])`, and
asserts against concrete values: at least one returned body's
`to_query().evaluate()` result, sorted by name, equals
`['H1','H1','H2','H2','H3','H3','H4','H4','H5']` exactly, and that
candidate's `score()` equals `RuleScore(support=9, confidence=0.36)`.

Verification: `pytest test/krrood_test/test_eql/test_rule_mining/ -v`, full
krrood suite green (same pre-existing environment-only gaps excluded as
item 2: `dot`/Graphviz binary, `jpt`, mypy's `librt`,
`semantic_digital_twin` — none touch `rule_mining`), self-containment
verified, `scripts/format_docstrings.py` run on new/changed files.

## `partnet-remote-access` — implementation plan (approved)

Depends on `engine-tests-synthetic` (PR #28, branch `engine-tests-synthetic`,
still open/draft). `check_dependency_readiness.py` reports `"is_ready": false`;
this item stacks on it anyway, per this repo's normal stacked-PR workflow and
the same call `engine-support-confidence` and `engine-tests-synthetic` each
recorded. PR #29.

**Ground truth verified on neem-4, since nobody with SSH access had confirmed
it before.** Access needs the `Uni Bremen` NetworkManager VPN
(`nmcli con up "Uni Bremen"`); without it SSH to port 22 times out.

The roadmap's category counts are *exactly* right — 345 `StorageFurniture`, 95
`Table`, 84 `Faucet` — verified by reading `model_cat` out of all 2218
`meta.json` files. 2218 models, 8.1GB, 46 distinct categories.

`partnet_meta` holds `all_ids.txt` (2347 `id,category` rows) and
`well_formed.txt` (2218 rows, exactly the on-disk model count) — so the on-disk
corpus *is* the well-formed subset. `StorageFurniture` appears 346 times in
`all_ids.txt` but 345 on disk: one model is not well-formed and is absent, so
sizing a run off `all_ids.txt` is off by one on this category.

`semantics.txt` is three columns (link name, motion kind, semantic label).
Across `StorageFurniture`: `rotation_door` 445 / `drawer` 349 /
`furniture_body` 345 / `translation_door` 28 / `wheel` 4 / `caster` 4 /
`board` 1; motion kinds `hinge` 454 / `slider` 377 / `heavy` 342 / `static` 3;
URDF joint types `revolute` 446 / `prismatic` 377 / `fixed` 345 /
`continuous` 8. One `furniture_body` per model — exactly one case/root body
each. `drawer`↔`slider`/`prismatic` and `rotation_door`↔`hinge`/`revolute` map
cleanly onto `Drawer(HasMechanicalJoint)` and `Door(HasHandle,
HasMechanicalJoint)`.

**A constraint this puts on `partnet-eql-domain-model`:** there is no `handle`
label anywhere in `semantics.txt`, so `HasHandle.handle` cannot be populated
from *these* labels.

.. note::
   Corrected while planning `partnet-eql-domain-model`: the conclusion
   originally drawn here — that handles must therefore be mined from geometry or
   the kinematic tree — was wrong. It generalised a fact about `semantics.txt`
   to the whole dataset. `mobility_v2.json` and `result.json` carry a richer
   part vocabulary in which handles are richly annotated: 318 of 345
   StorageFurniture models contain a `handle` part, and 326 of 384 `drawer`
   parts have a `handle` child. See that item's own section below.

**Why the loader had to change.** `sapien` is installed nowhere on neem-4 and
no `SAPIEN_ACCESS_TOKEN` exists in that environment, but `load()`
unconditionally calls `sapien.asset.download_partnet_mobility(...)`, and
`token` is an eager `field(default_factory=lambda: os.environ[...])` raising
`KeyError` at *construction* even when the corpus is already on disk. So the
loader could not read neem-4's corpus at all. This answers the plan's standing
question about `SAPIEN_ACCESS_TOKEN`: the token is not genuinely needed to read
an on-disk corpus, but the code demanded one regardless.

Verified token-free and sapien-free on model 35059: `URDFParser.from_file` with
`FileUriResolver(base_directory=...)` yields 3 bodies and 2 connections
(`FixedConnection`, `RevoluteConnection`), and `_apply_semantics_to_world`
yields 2 annotations (`PartNetFurniture`, `PartNetRotationDoor`) — the latter
being exactly the flat 1:1 rename this plan exists to replace. Incidentally:
`FileUriResolver(base_directory=...)` is load-bearing (without it mesh paths
resolve against the current working directory and `URDFParser` dies on
`textured_objs/original-21.obj`), and `available_model_ids` is a `@property`,
not a method.

**Mechanism: a Docker image on neem-4** (the user's call, over an
rsync-the-working-tree alternative). The image carries the *environment* only —
`python:3.12-slim` plus `git` plus `krrood/requirements.txt` and
`semantic_digital_twin/requirements.txt`, deliberately not `sapien`. The repo
is shallow-cloned at container start, so the branch is a run-time parameter and
a code change needs no rebuild (clone measured at 4s / 155M). The corpus is
bind-mounted read-only, so a run can never corrupt it.

```
ssh tom_sch@neem-4 docker run --rm \
  -v /raid/users/tom_sch/datasets/partnet-mobility-dataset:/data/partnet-mobility-dataset:ro \
  partnet-mining:latest <branch> <module> [args...]
```

Verified on the host: Docker 28.5.1, `tom_sch` in the `docker` group, 784G free
on `/`, container reaches the network, dataset mounts read-only (2218 models
visible, `touch` denied).

**New files.** `scripts/partnet_mining/{Dockerfile,entrypoint.sh,README.md}`;
`semantic_digital_twin/src/semantic_digital_twin/adapters/partnet_mobility_dataset/remote_execution.py`
with `RemoteExecutionConfiguration`, a `CommandRunner` protocol,
`SubprocessCommandRunner`, and `RemoteMiningJob` — dataclasses throughout, the
command runner injected so the tests assert on the constructed argument vector
and need neither SSH, VPN, nor Docker.

**Changed.** `loader.py`: `token` becomes `Optional[str]` defaulting via
`os.environ.get(...)`; the post-download half of `load()` is extracted into
`load_from_directory(model_id)`, which `load()` then delegates to; `load()`
raises an explicit error when the token is absent or `sapien` is not
importable, instead of today's `KeyError`/`NameError`. Behaviour-preserving for
the existing token-holding path.

**Tests first** (AGENTS.md): the argument vector `RemoteMiningJob` builds,
asserted against concrete expected values with a fake `CommandRunner` (runs in
CI); constructing `PartNetMobilityDatasetLoader` with `SAPIEN_ACCESS_TOKEN`
deleted no longer raises (runs in CI — the regression test for the eager
`default_factory`); and an end-to-end `load_from_directory` test asserting the
concrete 35059 values above, skipped unless the corpus is reachable.

**Deviation flagged at approval time.** The standing convention says CI-facing
tests keep the `SAPIEN_ACCESS_TOKEN`-gated `skipif` pattern. The corpus test
keeps that *shape* (env-var-gated `skipif`) but gates on a corpus/host variable
instead, because the remote path provably does not need that token — gating on
it would be a skip condition that no longer means what it says.

**Open risk, unverified at approval time.** The two requirements files were not
actually installed inside a container during planning (a multi-minute build).
`venv-sage` on the same host already resolves this dependency set under Python
3.12, so the set is known-good on this machine; if the image build fails, that
is where.

**Verification.** `pytest test/semantic_digital_twin_test/test_adapters/ -v`;
one real end-to-end run reproducing the 35059 numbers through `RemoteMiningJob`;
`scripts/format_docstrings.py` on new/changed files; `krrood` untouched, so
self-containment is unaffected.

## `partnet-eql-domain-model` — implementation plan (approved)

Depends on `partnet-remote-access` (PR #29), still open/draft;
`check_dependency_readiness.py` reports `"is_ready": false`. Stacked on it
anyway, the same call every prior item in this plan recorded. PR #30.

**Correction to the `partnet-remote-access` section above.** That section states
there is no `handle` label anywhere and concludes `HasHandle.handle` must be
mined from geometry or the kinematic tree. That was true of `semantics.txt`
alone and was wrongly generalised to the dataset. `mobility_v2.json` and
`result.json` carry a much richer part vocabulary: **318 of 345**
StorageFurniture models contain a `handle` part, and **326 of 384** `drawer`
parts have a `handle` child. The earlier claim is struck.

**`mobility_v2.json` is the join key nobody had used**, and nothing in the
repository reads it. Per URDF link it gives the link index, the motion kind, a
part-level name, the fine-grained parts and the parent link — for example
`link=1 joint=slider name='drawer'` with parts `drawer_front`, `drawer_side`,
`drawer_back`, `drawer_bottom` and `handle`.

**PartNet carries two disagreeing vocabularies**, which is the misalignment this
item exists to characterise. `semantics.txt`'s label against `mobility_v2.json`'s
name over all 345 StorageFurniture models: `rotation_door`->`cabinet_door` 414;
`drawer`->`drawer` 269; `furniture_body`->`shelf` 159; `furniture_body`->
`cabinet_frame` 151; `drawer`->`drawer_box` 47; **`drawer`->`handle` 32**;
**`furniture_body`->`drawer_front` 26**; `rotation_door`->`cabinet_door_surface`
24. The last two bolded rows are genuine annotation inconsistencies.

**Handle availability tracks the SemDT classes that need one:** `rotation_door`
410/445 (92%), `drawer` 299/349 (86%), `translation_door` 20/28 (71%),
`furniture_body` 1/345 (0%). So `Drawer(HasHandle, HasMechanicalJoint)` is
populatable for most PartNet drawers and `Door(...)` for most rotation doors,
while the case correctly has none — partial alignment, worth mining rather than
hard-coding.

**What the miner could already do, verified on real data rather than assumed.**
`DataclassOnlyIntrospector` discovers only declared dataclass fields, so a
`Body` is a dead end for mining: its `parent_connection` and
`child_kinematic_structure_entities` are properties over `self._world`.
`Connection` exposes `parent`/`child` and an annotation exposes `root`, so both
point *at* bodies. Heterogeneous joins already work — `PartNetRotationDoor` and
`Connection` closed on a shared body scored support=6, confidence=0.11 over
three models — and multi-hop works too (`annotation.root.visual`), so #28's
noted same-named-class collision in `krrood/class_diagrams` did not reproduce on
these types.

Exactly one capability was missing: `RuleMiner.mine()` seeded a second variable
only of the head's own type, while this domain needs one of a *different* type.

**krrood change (deliberate, and the user's call over driving search from the
bridge):** `rule_mining/miner.py` gains auxiliary seed domains keyed by type,
generalising `SelfJoinSeedStep` into a seed step naming the domain it draws
from. Head-type self-join stays the default, so #28's planted-rule test is
unchanged and acts as the regression guard. The capability is domain-agnostic,
so krrood stays self-contained.

**semantic_digital_twin change:** new
`adapters/partnet_mobility_dataset/domain_model.py` — a `mobility_v2.json`
parser plus flat, fully-minable dataclasses, every relation a declared field so
one hop reaches it: `PartNetMotionKind`/`PartNetSemanticLabel` as `StrEnum`s
over the real vocabularies, `PartNetPart`, `PartNetLink` (semantic label, motion
kind, part name, parts, the `World` body, its parent `Connection`, parent link)
and `PartNetModel`.

**Demo:** an entry point runnable through #29's `RemoteMiningJob`, mining over a
configurable number of StorageFurniture models and printing each pattern's
support/confidence together with the distribution of `semantic_label` among
matching links.

**Flagged at approval time.** Wave 1's miner scores rule *bodies*, not
implications — it has no notion of a consequent. The label distribution is
therefore a thin reporting layer added on the SemDT side, the minimum that makes
a mined pattern interpretable as PartNet-drawer versus SemDT-`Drawer`. A proper
conclusion-scoring abstraction probably belongs in krrood later; it is not built
here, and `partnet-rdr-bridge` is the item that turns conclusions into RDR
proposals.

**Open risks.** The seeding change amends wave-1 code while #26-#28 are still in
review. World-loading cost is unmeasured — each model runs convex decomposition
over its meshes; three were fine in a probe, 345 may not be, so the model count
is a parameter and a mesh-free path built straight from `mobility_v2.json` plus
the URDF is the fallback. Some mined patterns will restate the projection; if
everything comes back trivial that is a finding to report rather than hide.

**Tests first:** an auxiliary-domain mining case over the existing synthetic
fixture with #28's planted-rule test unchanged; domain-model parsing against a
`mobility_v2.json` added to #29's synthetic model fixture, so it runs in CI with
no corpus; a corpus-gated test asserting model 45162's drawer link carries a
handle part and a prismatic connection; and a demo assertion that the mined
`slider`-plus-handle pattern recovers a `drawer` share matching the measured 86%
within a stated tolerance.

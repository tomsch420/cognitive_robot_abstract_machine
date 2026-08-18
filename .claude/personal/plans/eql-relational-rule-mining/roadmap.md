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

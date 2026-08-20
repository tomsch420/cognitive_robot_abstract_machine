## `partnet-eql-domain-model` (PR #30, stacked on #29) — demo working, awaiting review

### Done

- **krrood** `miner.py`: auxiliary seed domains (`SeedDomain`/`SeedStep`), instantiated
  atoms (`ConstrainStep` + `candidate_values`), and `MinedRule` (body + plan) with
  `describe()`. #28's planted-rule test unchanged and passing.
- **semantic_digital_twin** `domain_model.py`: `mobility_v2.json` parser (nothing read it
  before), `PartNetMotionKind`/`UrdfJointType`/`StorageFurnitureLabel`, `PartNetPart`,
  `PartNetLink`, `PartNetModel` with both `from_world` and mesh-free `from_dataset`.
- **experiments** `partnet_rule_mining_demo.py`: mines and reports rules with the
  `semantics.txt` label distribution per pattern.
- Tests: 18 krrood mining + 11 domain model, all green against the real corpus.
- **Demo verified**: 25 models / 84 links / depth 2 / 194s. Joint type separates
  drawer from door 100% on that slice; `has_handle` deliberately does not (55/45), which
  is correct since Drawer and Door both mix in HasHandle.

### Two wave-1 gaps found and fixed here

1. The search generated only 2 of AMIE's 3 operators — `constrain_variable_to_value`
   landed in #26 but #28's loop never emitted instantiated atoms, so no rule about a
   *value* was reachable.
2. A mined rule could not be read back: an EQL comparator renders as `==` alone and its
   `Literal` hides the wrapped value, so conditions printed as `joint_type AND == AND ==`.
   Hence `MinedRule` carrying the plan.

### Known limit, not attempted

The miner does not scale. Depth 4 — needed for a two-attribute conjunction, since every
extend and every constrain each cost an atom — did not finish in 8 minutes at 40 models.
The search re-materializes and re-scores every candidate from its plan, two query
evaluations each. Fixing it (indexing, incremental scoring, reusing materialized bodies)
is engine work. **Asked the user whether to take that on or leave #30 for review; no
answer yet — do not start it unprompted.**

### Performance measures already taken (also just better modelling)

- `has_handle`, `parent_semantic_label`, `parent_part_name` stored flat rather than a
  property and a link hop.
- `body`, `connection`, `parent_link` are `init=False` so attribute discovery skips them —
  a `Body` otherwise leads the search into mesh collections.
- Search runs inside `MonitoredRegistry().disabled()`; EQL captures a call stack per
  variable construction.
- Demo uses `from_dataset`: loading worlds runs convex decomposition over every mesh and
  dominated everything (8 models took 6+ min and never reached mining).

### Watch out

- Wave-1 operators MUTATE in place and return the body; the new variable is
  `body.open_variables[-1]`.
- Piping a remote run through `tail` buffers all output until exit — write to a file on
  the host and poll it instead.
- Report filters out rules that constrain only a seeded companion variable: they leave
  the head unconstrained and report the corpus-wide distribution.
- Keep PR #30 draft after every push.

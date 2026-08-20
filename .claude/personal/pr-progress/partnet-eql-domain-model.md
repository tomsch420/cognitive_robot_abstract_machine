## `partnet-eql-domain-model` (PR #30, stacked on #29)

Expose a loaded PartNet `World` as EQL-minable structure, and produce the first
demo of rules mined from real data. Approved plan in `roadmap.md`; findings in
PR #30's description.

### Plan

1. `krrood/.../rule_mining/miner.py` — auxiliary seed domains keyed by type, so a
   second variable of a *different* type can be seeded. Head-type self-join stays
   the default; #28's planted-rule test is the regression guard.
2. `semantic_digital_twin/.../partnet_mobility_dataset/domain_model.py` — parse
   `mobility_v2.json` (nothing reads it today) into flat minable dataclasses:
   `PartNetMotionKind`/`PartNetSemanticLabel` StrEnums, `PartNetPart`,
   `PartNetLink`, `PartNetModel`.
3. Demo entry point runnable through #29's `RemoteMiningJob`, printing each mined
   pattern's support/confidence plus the `semantic_label` distribution.

TDD: tests before the code they cover.

### Done

- Branch + draft PR #30; plan.yaml and roadmap.md recorded.
- **Corrected the wrong handle claim in all three places it had spread**: PR #29's
  description, the roadmap's `partnet-remote-access` section, and this note's
  predecessor. Handles ARE in the dataset (318/345 models), just not in
  `semantics.txt`.

### Key evidence (measured, not assumed)

- `mobility_v2.json` is the join key: per link it gives motion kind, part-level
  name, fine-grained parts (incl. `handle`), parent link.
- Two disagreeing PartNet vocabularies. Notably `drawer`->`handle` 32 and
  `furniture_body`->`drawer_front` 26 are real annotation inconsistencies.
- Handle coverage: rotation_door 92%, drawer 86%, translation_door 71%,
  furniture_body 0%.
- `Body` is a DEAD END for mining (`DataclassOnlyIntrospector` sees only declared
  fields; its navigation is `@property` over `_world`). Mine from `Connection`
  or an annotation, both of which point at bodies.
- Heterogeneous joins and multi-hop both already work on real data.

### Next

- Write the three test files first, then miner change, domain model, demo.
- Measure world-load cost early; make model count a parameter. Mesh-free fallback
  (mobility_v2 + URDF only) if 345 models is too slow — raise before taking it.

### Watch out

- Wave 1 operators MUTATE in place and return the body; the new variable is
  `body.open_variables[-1]`, not the return value. Cost me a probe.
- Keep PR #30 draft after every push.
- Run tests in the neem-4 container; this machine cannot (see #29's notes).

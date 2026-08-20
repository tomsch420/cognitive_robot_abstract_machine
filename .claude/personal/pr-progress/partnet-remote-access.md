## `partnet-remote-access` (PR #29, stacked on #28)

Docker-on-neem-4 workflow for running mining code against the PartNet-Mobility
corpus. Approved plan in `roadmap.md`; full detail in PR #29's description.

### Done

- **All code written and pushed.** `scripts/partnet_mining/{Dockerfile,entrypoint.sh,README.md}`;
  `remote_execution.py` (`RemoteExecutionConfiguration`, `CommandRunner` protocol,
  `SubprocessCommandRunner`, `RemoteMiningJob`); `loader.py` (`token` optional,
  `load_from_directory`, non-mutating `_urdf_with_completed_limit_tags`);
  `MissingSapienAccessTokenError`.
- **Tests green on neem-4 against the real corpus: 12 passed, 1 skipped.**
  The skip is the pre-existing `test_loader`, which genuinely needs a download token.
  Includes the gated 35059 test, which confirmed the hand-measured values.
- **Image builds.** `partnet-mining:latest`, 3.8GB. The heavy dependency set
  (pygraphviz, python-fcl, casadi, ortools, coacd...) installs fine on python:3.12-slim
  with build-essential + libgraphviz-dev.
- **Second loader bug found and fixed:** `_add_missing_information_to_limit_tags` did
  `tree.write(file_path)` — it mutated the dataset in place, which the read-only mount
  rejects outright. Now returns a string; covered by a bytes-before/after test.

### Next

- Finish the end-to-end container run. Blocked twice on undeclared deps, each fixed
  in the Dockerfile: `objgraph` (root dev extra) and `giskardpy_bullet_bindings`
  (giskardpy's). Rebuild is running; re-run the container test invocation after it.
- If more undeclared imports surface, keep adding them to the Dockerfile's explicit
  list rather than pulling in whole workspace packages.

### Undeclared dependencies discovered (worth reporting upstream)

`semantic_digital_twin` imports three packages neither requirements file declares:
`mujoco` (physics_simulators), `giskardpy_bullet_bindings` (giskardpy), and
`objgraph` (root dev extra, via `test/conftest.py`). Any environment built from
`semantic_digital_twin/requirements.txt` alone cannot import the package.

### Watch out

- `.gitignore` has a blanket `*.txt`; the synthetic fixture's `semantics.txt` needed
  an explicit negation, following the existing `set-up-clone` precedent.
- Joint limits live at `connection.raw_dof.limits.upper.velocity`, not on the connection.
- `DataclassException` subclasses must implement **both** `error_message` and
  `suggest_correction` — omitting the latter fails at construction, not import.
- `FileUriResolver(base_directory=...)` is load-bearing for mesh paths.
- Local machine cannot run these tests (workspace not installed, PEP 668). Run them on
  neem-4's `venv-sage` with `PYTHONPATH` shadowing plus `--noconftest`.
- Keep PR #29 draft after every push.

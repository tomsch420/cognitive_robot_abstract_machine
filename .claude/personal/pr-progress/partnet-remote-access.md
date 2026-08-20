## `partnet-remote-access` (PR #29, stacked on #28) — complete, awaiting review

Docker-on-neem-4 workflow for running mining code against the PartNet-Mobility
corpus. Approved plan in `roadmap.md`; full detail in PR #29's description.

### Done — everything in the approved plan, verified end to end

- `scripts/partnet_mining/{Dockerfile,entrypoint.sh,README.md}`; `remote_execution.py`
  (`RemoteExecutionConfiguration`, `CommandRunner`, `SubprocessCommandRunner`,
  `RemoteMiningJob`); `loader.py` (`token` optional, `load_from_directory`,
  non-mutating `_urdf_with_completed_limit_tags`); `MissingSapienAccessTokenError`.
- **Real end-to-end run from the laptop**: `RemoteMiningJob(...).run()` -> SSH ->
  `docker run` -> clone -> pytest against the read-only corpus -> exit code 0.
- Targeted tests in the container: 12 passed, 1 skipped (the skip is the pre-existing
  `test_loader`, which needs a download token). Includes the gated 35059 test.
- Whole `test_adapters/`: 91 passed. The 3 failures + 11 errors are ROS/simulator
  environment gaps and reproduce identically on base branch `engine-tests-synthetic`
  in the same container — confirmed pre-existing, not caused by this PR.
- Image `partnet-mining:latest`, 4.07GB.

### Second loader bug found while building this

`_add_missing_information_to_limit_tags` did `tree.write(file_path)` — it mutated the
dataset in place, which the read-only mount rejects outright and which would silently
modify the user's corpus otherwise. Now returns a string for `URDFParser(urdf=...)`;
covered by a bytes-before/after test.

### Worth doing separately (not this PR)

`semantic_digital_twin` cannot be imported from its own `requirements.txt`: it needs
`giskardpy` (+ `giskardpy_bullet_bindings`), `mujoco` (declared only by
`physics_simulators`), and `objgraph` (root dev extra, via `test/conftest.py`). The
image works around it; the requirements files themselves should probably be fixed.

### Next

- Nothing outstanding. PR #29 is a draft awaiting author review.
- Follow-on item is `partnet-eql-domain-model`, which now has real structure to design
  against — note especially that **no `handle` label exists** in `semantics.txt`, so
  `HasHandle.handle` has to be mined from geometry or the kinematic tree.

### Watch out

- `.gitignore` has a blanket `*.txt`; the synthetic fixture's `semantics.txt` needed an
  explicit negation, following the existing `set-up-clone` precedent.
- Joint limits live at `connection.raw_dof.limits.upper.velocity`, not on the connection.
- `DataclassException` subclasses must implement both `error_message` and
  `suggest_correction` — omitting the latter fails at construction, not import.
- `FileUriResolver(base_directory=...)` is load-bearing for mesh paths.
- This machine cannot run these tests (workspace not installed, PEP 668). Run them on
  neem-4 via the container, or on `venv-sage` with `PYTHONPATH` shadowing + `--noconftest`.
- Keep PR #29 draft after every push.

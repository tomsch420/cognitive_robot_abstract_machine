## `partnet-remote-access` (PR #29, stacked on #28)

Establish the SSH/Docker workflow for running krrood mining code against the
PartNet-Mobility corpus on neem-4. Approved plan is in `roadmap.md` under
"`partnet-remote-access` — implementation plan (approved)"; full detail also in
PR #29's description.

### Plan

1. `scripts/partnet_mining/{Dockerfile,entrypoint.sh,README.md}` — environment-only
   image (`python:3.12-slim` + git + both requirements files, **no sapien**);
   entrypoint shallow-clones the requested branch, sets `PYTHONPATH`, execs the module.
2. `.../partnet_mobility_dataset/remote_execution.py` — `RemoteExecutionConfiguration`,
   `CommandRunner` protocol, `SubprocessCommandRunner`, `RemoteMiningJob`. Runner is
   injected so tests assert on the built argument vector (no SSH/VPN/Docker in CI).
3. `loader.py` — `token` to `Optional[str]` via `os.environ.get`; extract
   `load_from_directory(model_id)` from `load()`; explicit error when token/sapien
   missing.

TDD: tests before the code they cover, concrete expected values, no not-None checks.

### Done

- VPN + SSH access confirmed (`nmcli con up "Uni Bremen"` is required).
- Corpus verified: 2218 models / 8.1GB / 46 categories; roadmap counts exact
  (345 StorageFurniture, 95 Table, 84 Faucet).
- `partnet_meta` = `all_ids.txt` (2347 rows) + `well_formed.txt` (2218 = on-disk).
  StorageFurniture is 346 in all_ids, 345 on disk -> one not well-formed.
- StorageFurniture semantics/joint vocabulary surveyed. **No `handle` label exists**
  -> `HasHandle.handle` must be mined, not read off labels. Matters for the next item.
- Loader blocker found and characterised: no sapien and no token on neem-4, `load()`
  always calls `download_partnet_mobility`, `token` KeyErrors at construction.
- Token-free/sapien-free load proven on model 35059: 3 bodies, 2 connections
  (Fixed, Revolute), 2 annotations (PartNetFurniture, PartNetRotationDoor).
- Docker on neem-4 verified: 28.5.1, docker group, 784G free on `/`, outbound net,
  dataset mounts `:ro` (writes denied), shallow clone 4s / 155M.
- Branch + draft PR #29 opened; plan.yaml and roadmap.md recorded.

### Next

- Write the three test files first (argument vector; loader constructs without token;
  gated 35059 end-to-end).
- Then Dockerfile + entrypoint + README, `remote_execution.py`, loader change.
- Build the image on neem-4 and do one real end-to-end run reproducing the 35059
  numbers. **This is the main unverified risk** — the two requirements files were
  never installed in a container during planning.
- `scripts/format_docstrings.py` on new/changed files; keep PR #29 draft after pushes.

### Watch out

- `FileUriResolver(base_directory=...)` is load-bearing — without it URDF mesh paths
  resolve against cwd and the parse dies on `textured_objs/original-21.obj`.
- `available_model_ids` is a `@property`, not a method.
- `AGENTS.md` forbids assistant co-author trailers; commits use `Made with the help
  of Claude` as a plain line.
- If a container fails to start on neem-4, check `df -h /` first (images live on `/`).

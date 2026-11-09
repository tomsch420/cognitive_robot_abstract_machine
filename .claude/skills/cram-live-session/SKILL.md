---
name: cram-live-session
description: Drive a simulated CRAM robot step by step from a persistent IPython kernel while the developer watches it live in cramera and says what the next step is. Use when the developer wants to "work in the IPython workflow", "tell you each step", "watch it in cramera while you code", replay or undo a step, or debug an action (e.g. a RoboCasa task) interactively.
allowed-tools: Bash, Read, Write, Edit
---

# CRAM live session

The developer watches the robot in cramera and says what the robot does next. You write
each step as a *cell*, show its code, perform it in a kernel that stays alive between
steps, and report what happened. The kernel keeps the world, the robot, the simulation and
every moment before a cell, so a step can be undone, retried or replayed in seconds instead
of rebuilding the scene.

The data structures live in CRAM (`experiments.live_session`, and the snapshot of a
stepped MuJoCo simulation in `semantic_digital_twin.adapters.multi_sim`); the scripts
next to this file drive the kernel. Code you write in cells follows `@AGENTS.md` only
loosely: cells are throwaway, but anything that proves itself moves into the codebase
under the full rules.

## The rules of the loop

1. **One step per instruction.** Do what the developer asked, then stop and wait. Never
   chain further robot steps on your own, even when the next one seems obvious. If an
   instruction has several parts ("open the gripper, put a finger left of the lever, push
   right"), it is one step; perform its parts in one cell.
2. **Show the code before you perform it.** Every motion cell goes into the chat as a code
   block first, then gets performed. Measuring code may be summarised in a sentence.
3. **Report from the reports.** Every cell ends with `session.report(...)`; quote the
   status, tool and base positions, and say in a sentence what that means for the
   instruction (reached, stopped short by how much, what failed).
4. **When a step fails, diagnose before retrying.** Measure with a raw run (what is near,
   which joint is at its limit, which body blocks) and offer the developer concrete
   options with a recommendation. Do not loop through blind retries.
5. **Undo instead of rebuilding.** To retry a step, `session.revert_last_cell()`; to show
   steps again, `session.replay(i, globals())`. Name a checkpoint the developer will want
   to come back to with `session.take_snapshot(name)`.
6. **Keep the developer's words.** Cell titles say what the developer asked for, in their
   words; they are what the history shows.

## Setting up

Defaults are the neem-4 RoboCasa container (`tom-cram-robocasa-experiments`, CRAM in
`/opt/cram`, environment `/root/cramera_live_env.sh`); every script reads its settings
from `LIVE_*` environment variables, listed at its top.

1. Start the kernel and the viewer in the container (each only if not running):
   `ssh SERVER docker exec -d CONTAINER bash /opt/cram/.claude/skills/cram-live-session/scripts/start_session.sh`
2. The developer opens the viewer through the tunnel on their machine
   (`~/.local/bin/cramera-neem4`, then `http://localhost:8711/?live=http://127.0.0.1:8765`).
3. Copy `templates/robocasa_session.py` to your scratchpad, set the task and robot in its
   first cell, and run it raw: `scripts/live_kernel.sh raw FILE`. It composes the episode,
   starts the physics, installs the viewer lock workaround and creates `session` with a
   snapshot `composed`.

After a kernel restart everything in it is gone, snapshots included; run the template
again.

## Performing a step

Write the cell to a file in your scratchpad, starting with `# %% cell <title>`:

```python
# %% cell put one finger left of the lever, push it right
...
session.report("pushed right", outcome)
```

then `scripts/live_kernel.sh perform FILE` (several cells: all of them, or
`perform FILE 2:4`). The output streams back as the kernel prints it. A run that is still
going after `LIVE_WAIT_SECONDS` keeps running; follow it with `live_kernel.sh wait NAME`.

Use `live_kernel.sh raw FILE` for measuring and for set-up code: a raw run is not kept in
the history, so replays do not repeat it.

## Data structures

`experiments.live_session.session`:

| | |
|---|---|
| `LiveSession(simulation, tool_frame, robot_base)`, `LiveSession.from_episode(episode)` | The session; `session` in the kernel. |
| `session.report(label, outcome=None) -> StepReport` | Prints one aligned line (label, status, tool and base position) and keeps it with the running cell. |
| `session.perform(cell, namespace)` | Snapshot, then run the cell in `namespace` (the kernel's `globals()`); `perform_cells.py` calls it. |
| `session.history: list[PerformedCell]` | Every performed cell: `cell`, `snapshot_before`, `reports`, `finished` (false if it raised). |
| `session.revert_last_cell() -> Cell` | Back to the moment before the last cell, which is dropped. |
| `session.replay(first_cell, globals())` | Back to the moment before history entry `first_cell`, then perform it and every later cell again. |
| `session.take_snapshot(name)`, `session.restore_snapshot(name)` | Named checkpoints in `session.snapshots`. |
| `StepReport.status` | `StepStatus.SUCCEEDED` (`ok`), `StepStatus.NO_PLAN` (empty), or the failure's class name. |

`experiments.live_session.cells`: `Cell(title, source)`, `CellScript.from_file(path)`,
`CellMarker.START` (`# %% cell `).

`semantic_digital_twin.adapters.multi_sim`: `MujocoSim.take_snapshot() ->
MujocoSimulationSnapshot` and `MujocoSim.restore_snapshot(snapshot)` restore the physics
exactly (positions, velocities, controls, time), the world state and the servo set points;
the physics then continues as it did after the snapshot. A snapshot does not survive a
change to the world's model (`SnapshotDoesNotFitSimulationError`).

## Useful pieces for cells

- World-frame box of a body: `body.collision.as_bounding_box_collection_in_frame(world.root).bounding_box().to_array_bounds()` (`.lower`, `.upper`); pass the robot's root as the frame for a footprint.
- A MuJoCo joint's anchor and axis: `mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)`, then `data.xanchor[j]`, `data.xaxis[j]`, `data.qpos[m.jnt_qposadr[j]]`, `m.jnt_range[j]` (`m, data = episode.simulation.simulator._mj_model, episode.simulation.simulator._mj_data`).
- Arm only, base fixed: `episode.robot.mobile_base.full_body_controlled = False`; set it back to `True` before a `NavigateAction` that should move the base.
- Allow contact: `with episode.allowances.bodies_may_touch(hand_bodies, other_bodies): ...`.
- Motions: `ReachThroughFreeSpaceAction(goal=pose, arm=episode.arm)` (path through free space), `FollowToolCenterPointPathAction(target_locations=PoseTrajectory([...]), arm=episode.arm)` (straight), `NavigateAction(target_location=pose)`, `MoveTorsoAction(torso_state=TorsoState.LOW)`, `SetGripperAction(configuration=gripper.default_configuration(GripperState.OPEN))`; run them with `episode.run([...])`, which returns the `PlanOutcome` to report.
- Joint limits and what is near each arm link are the first things to measure when a motion stops with `MotionMadeNoProgress`.
- Don't name kernel variables `m` or `d` in measuring cells: they are the MuJoCo model and data.

## Turning a session into an action

When the developer asks for an action from what worked, write it bottom-up from the cells
that worked, not from existing actions of the same kind. See
`experiments/robocasa_atomic_tasks/sink_actions.py` (`LiftTheHandOutOfTheSinkAction`,
`TurnOnTheFaucetWithAFingerAction`) for the shape:

- **Parameters** are fields defaulting to the values that worked, expressed relative to the
  objects involved (a handle's extent, a joint's axis), never in world coordinates, so they
  can later be varied and learned through the probabilistic backend.
- **Preconditions** are an EQL condition from `preconditions()` (mixin `HasPreconditions`),
  checked as the first step; give each predicate one operand as a `variable(...)` so the
  condition is symbolic. A false one fails the action with `PreconditionsNotMet`.
- **Tolerances** the session silently accepted (a motion that reported
  `MotionMadeNoProgress` but was close enough) become explicit parameters.
- Steps whose poses depend on where earlier steps left the robot are given as statements
  (`a(StepAction)(...)`), which are expanded only when they start; plain nodes are expanded
  when the plan is compiled.
- Collision allowances that must hold from one step to the next belong to the outer
  action's `create_temporary_collision_rules`, not to each step.
- Load the new module into the kernel from the worktree (`importlib.util.spec_from_file_location`),
  run the action as a cell from the snapshot the session began at, and let the developer
  watch it before writing the tests.

## When something goes wrong

- **No output and the run does not end**: `live_kernel.sh` warns when the kernel sleeps
  without output. That is a deadlock, almost always between cramera's publisher lock and
  the world's lock. `live_kernel.sh interrupt` raises `KeyboardInterrupt` in the kernel and
  the traceback shows where it waited. The template's lock cell prevents it; if it happens
  anyway, report the traceback to the developer.
- **`docker exec` hangs** on some servers after the command finished: the scripts always
  run detached and read a log instead. Do the same for anything you run by hand.
- **A restore is refused** with `SnapshotDoesNotFitSimulationError`: the world's model
  changed (an object was spawned or removed); take new snapshots after such a change.
- **A pushed or pulled joint moves only after the motion**: the world follows the
  kitchen's joints during a plan only with `episode.leaves_the_environment_to_the_physics = False`,
  which the template sets.
- **The live view shows nothing new**: check the viewer log in the session directory, and
  that the tunnel on the developer's machine still runs.

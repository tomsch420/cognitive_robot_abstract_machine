# Montessori shape-sorting demo — handover

Status as of 2026-07-17. Full test suite green (see "Running things" below).

## What this is

A semantic digital twin scene of a Montessori shape-sorting board: a table with a
board that has one hole per shape category (cube, cylinder ×2, triangular prism,
rectangular prism, disk) plus a sphere with no matching hole, an HSRB robot that sorts
the loose shapes into their holes via real CRAM motion planning, live RViz
visualization, and an optional MuJoCo physics simulation of the finished scene.

Run it with:

```
python -m experiments.montessori.montessori_demo
python -m experiments.montessori.montessori_demo --headless
```

Requires a sourced ROS 2 workspace and the `hsr_description` package; without the
latter the scene still spawns but no robot/sorting/MuJoCo happens (see the module
docstring in `montessori_demo.py`).

## File map

- **`world.py`** — builds the scene (`MontessoriWorld`): floor, table, board (with
  holes cut from its mesh), loose shapes, and `spawn_hsrb()`. Board mesh, hole
  detection, and the board's convex collision decomposition are all computed once at
  **module import time** and cached at module level (`_BOARD_MESH`, `_HOLE_FOOTPRINTS`,
  `_BOARD_COLLISION_PARTS`), not per-`MontessoriWorld()` instance.
- **`hole_geometry.py`** — detects true (non-rectangular) hole shapes from the board
  STL via trimesh boolean CSG, and cuts them fully through the board mesh.
- **`semantics.py`** — semantic annotations: `MontessoriShape`, `ShapeSortingBoard`,
  `ShapeSortingHole`, `NoMatchingHoleError` (a `DataclassException`).
- **`insert_shape_action.py`** — `InsertMontessoriShapeAction`, the CRAM action that
  picks up one shape and inserts it through its matching hole. Uses `MoveToReach` +
  Graph of Convex Sets (GCS) navigation maps with generative (`ProbabilisticBackend`)
  resolution of the robot's standing offset, not the older `NavigateAction` +
  `reachability_location()` costmap search.
- **`montessori_demo.py`** — the runnable demo: builds the world, spawns the robot,
  starts RViz/TF publishers, sorts all shapes, then runs a MuJoCo simulation of the
  finished scene with the robot's joints PD-held and the shapes given free joints so
  they're actually physics-movable.

## What's verified working (via real execution, not just plan-building)

- All 6 shapes get picked up and correctly inserted into their matching holes; the
  sphere is correctly skipped.
- RViz/TF publishing and the full `main()` entry point run end-to-end.
- MuJoCo simulation of the finished scene: robot arm/head/base held stable via PD
  actuators + joint damping (no more free-fall or wheel-spin), shapes given
  `Connection6DoF` joints so they actually move under gravity instead of staying
  welded in place.

## Known limitations (root cause understood, not fixed)

1. **Shapes don't fully fall through the holes in MuJoCo.** MuJoCo's default mesh
   collision uses the convex hull, which would erase the board's holes entirely; fixed
   by decomposing the board's collision geometry into ~40–50 convex pieces via CoACD
   (`_BOARD_COLLISION_DECOMPOSER` in `world.py`). This is a large, measured improvement
   (raycast-grid coverage of each hole's opening went from ~0% to ~73% average across
   all 6 holes) but not a perfect fit — a dropped shape gets caught on a residual,
   imperfectly-approximated lip rather than passing all the way through. The `disk_hole`
   (a narrow ~5mm slot) is the worst case, still mostly closed. Confirmed via direct
   MuJoCo contact inspection (`ncon`, `qvel`), not just position — this is a genuine
   geometric limit of decomposition-based collision at this hole size, not a bug.
   Next step if revisited: a hand-built cylindrical/slot collision insert per hole, or
   MuJoCo's SDF plugin (bundled but not wired into `semantic_digital_twin`'s MuJoCo
   adapter at all — would be new integration work).

2. **`hole_for()` matches by category only.** Two holes (`circular_hole_1`, 16mm
   radius, and `circular_hole_2`, 20mm radius) both categorize as `cylinder`, so a
   shape can get routed to the tighter-fitting same-category hole. Harmless for CRAM's
   kinematic placement (`PlaceAction` just teleports the shape), but relevant if you
   ever rely on exact hole/shape size matching.

3. **Giskard collision avoidance (`simulated_robot(collision_avoidance=True)`) breaks
   the pickup motion — currently left OFF.** Not a detected-and-cancelled violation
   (tested `cancel_if_collision_violated=False`, no change) and not simply "gripper
   can't touch the target" (tested adding the same gripper/target
   `AllowCollisionBetweenGroups` rule `GiskardLocationBackend` uses, no change). Traced
   to `coraplex/plans/executables.py:336`: the motion statechart is ticked up to
   `len(motion_mappings) * 2000` times and raises `MotionDidNotFinish` if it never
   reaches `is_end_motion()` — i.e. a **QP convergence timeout**, not a collision
   exception. Most likely cause: the ~40–50-piece CoACD-decomposed table/board geometry
   (see #1) gives `ExternalCollisionAvoidance` far more simultaneous distance
   constraints to satisfy than the original single mesh, and a shape resting flush on
   the table is an inherently tight-clearance grasp target. Next step if revisited: try
   a larger tick budget or looser `buffer_zone_distance` first (cheap); reducing the
   CoACD piece count for the table/board would help but trades away some of #1's
   hole-opening precision.

## Git state

`git log -1 -- world.py` shows it (and `hole_geometry.py`) already committed at HEAD
(`b2eabf659`) — someone has been checkpointing as we went. **Not yet committed:**

- Modified: `insert_shape_action.py`, `montessori_demo.py`, `semantics.py`
- New: `test/experiments_test/test_montessori_demo.py`,
  `test_montessori_hole_geometry.py`, `test_montessori_insert_shape_action.py`,
  `test_montessori_mujoco.py`
- New resource: `resources/board.stl`

Nothing has been committed by me during this session (per policy, only on explicit
request) — review and commit at your discretion.

## Running things

```
python3 -m pytest test/experiments_test/ -k montessori -v
```

35 passed, 1 skipped (`test_montessori_mujoco.py`'s own test is CI-gated). Module
import takes ~7–9s due to the CoACD decomposition running once at import time; this is
a one-time cost per process, not per test.

## Remaining open TODOs (not started)

From an earlier working-notes file (since removed):

- Give `montessori_demo.py` a configurable class for its main block instead of relying
  on module-level global constants.
- `world.py` could bind more of its scattered helper functions onto classes rather
  than leaving them as module-level functions.

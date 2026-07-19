# Montessori shape-sorting demo — handover

Status as of 2026-07-19. First run against a **real ROS 2 install, real HSRB, and real
CRAM/Giskard motion planning** (prior sessions only ever ran against a sandbox with no
`rclpy`, so most of this file used to describe untested, plan-building-only behavior).
Full test suite green except for two known, environment-specific gaps (see "Running
things"). A real bug found via this real execution (shapes always starting MuJoCo's
gravity simulation at the world origin, see "What's verified working") turned out to
explain most of what an earlier session had attributed to MuJoCo collision-decomposition
imprecision — fixed; 4/6 shapes now fall through on the first attempt instead of 1/6.

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

## Getting a real ROS 2 install in a sandbox without `packages.ros.org` access

`.github/docker/setup_workspace.py` (apt-based, against `packages.ros.org`) is what CI
uses and should keep working there. It does **not** work in a sandbox whose network
can't reach `packages.ros.org`: that host's own TLS certificate (served by its OSUOSL
mirror infrastructure) has no `packages.ros.org` SAN — `subject: CN=*.osuosl.org` — so
no client (`curl`, `apt`, anything) can complete a verified TLS handshake to it. This is
a real fault in that host's own certificate, not a local trust-store or proxy issue:
the same mismatch reproduces via direct SNI probing, independent of any HTTP(S) proxy.

Worked around it this session with a RoboStack (conda-forge) ROS 2 Jazzy install
instead, built entirely from `conda.anaconda.org`, which doesn't have this problem:

```
# micromamba (or mamba/conda) with channels robostack-jazzy + conda-forge:
ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-py-trees-ros ros-jazzy-xacro
compilers cmake pkg-config make ninja colcon-common-extensions python=3.12
```

Then `uv pip install --python <that env's python> -e ".[dev]"` to get the workspace
packages (coraplex, semantic_digital_twin, etc.) installed against that interpreter, and
`colcon build --merge-install` a workspace with the same repos
`setup_workspace.py` clones (`iai_maps`, `iai_robots`, `iai_pr2`, `hsr_description`,
`iai_tracy` (description only), `ros2_robotiq_gripper`,
`Universal_Robots_ROS2_Description`, `stretch_ros2`, `iai_tiago_description`,
`pmb2_robot`, `pal_gripper`, `robokudo_msgs`, `cram_ros2_packages`,
`iai_weiss_wpg_300-120-gripper`) plus `realsense-ros`'s `realsense2_description` (needed
transitively by Stretch's URDF; not itself in `setup_workspace.py`'s list, worth adding
there). `rclpy_message_converter` isn't packaged for ROS 2 Jazzy on either
`packages.ros.org` or robostack-jazzy; its source (`github.com/uos/rospy_message_converter`,
branch `jazzy`) is a plain `ament_python`/setuptools package, so
`pip install ./rclpy_message_converter` from a clone works directly.

**Do not `micromamba install mongodb` into that same env** — even though conda-forge
has it, its dependency resolution silently downgrades/removes
`ros-jazzy-ament-index-python` and `ros-jazzy-xacro` in the process (discovered the hard
way; recovered by reinstalling `ros-jazzy-desktop` + `ros-jazzy-xacro` +
`ros-jazzy-navigation2` + `ros-jazzy-py-trees-ros` again). If `robokudo_test`'s MongoDB
tests are needed, run `mongod` from a **separate** environment/download instead.

Also needed, unrelated to ROS: `apt-get install libegl1` (for PySide6/Qt GUI tests) and
`export QT_QPA_PLATFORM=offscreen` before running anything that imports
`PySide6.QtWidgets.QApplication` (`test/probabilistic_model_test/test_gui/`), or Qt
aborts the whole process (`Fatal Python error: Aborted`) trying to open a display that
doesn't exist. CI already sets this (see `ci_reusable.yml`); it just wasn't obvious
outside CI.

## File map

- **`world.py`** — builds the scene (`MontessoriWorld`): floor, table, board (with
  holes cut from its mesh), loose shapes, and `spawn_robot()` (generic over any
  `AbstractRobot` subclass, e.g. `HSRB`). Board mesh, hole
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
  `reachability_location()` costmap search. Every field of every action gets built
  through `underspecified(SomeActionClass)(...)`, not called directly — that class had a
  bare `a(SomeActionClass)(...)` (5 call sites) until this session, which always raised
  `AttributeError: type object 'X' has no attribute '_quantify_'` the moment the action
  plan was actually built (`a()`/`an()` require an already-built `Query`/`Match`
  instance, not a bare class) — untested until now for the same "needs real rclpy to
  even collect" reason as everything else here.
- **`montessori_demo.py`** — the runnable demo: builds the world, spawns the robot,
  starts RViz/TF publishers, sorts all shapes, then runs a MuJoCo simulation of the
  finished scene with the robot's joints PD-held and the shapes given free joints so
  they're actually physics-movable.

## What's verified working (via real execution against real HSRB + rclpy)

- `python -m experiments.montessori.montessori_demo --headless` runs end to end,
  without crashing, against a real HSRB and real CRAM/Giskard motion planning: builds
  the world, spawns the robot, attempts all 6 shapes (each up to
  `MAX_INSERTION_ATTEMPTS` times), correctly skips the sphere, runs the finished-scene
  MuJoCo simulation, and shuts down cleanly on `SIGINT` (no exceptions in the `finally`
  cleanup, no lingering threads/processes).
- RViz/TF publishing (`TFPublisher`, `VizMarkerPublisher`) construct and start without
  error against a real `rclpy` node.
- All 7 tests in `test_montessori_insert_shape_action.py` pass (see the `a()` →
  `underspecified()` fix above — they could not even be collected before this session).
- `test_sage10k.py` and `test_scalability.py` pass (4/4) — same "needs real rclpy to
  collect" class of test as above.
- Three real bugs were found and fixed by this end-to-end run, all previously
  unreachable without real rclpy + a real robot:
  - `_insert_all_shapes`'s retry loop didn't catch `PointOccupiedError` (raised by
    `_move_to_reach` when a jittered retry drop point lands outside the GCS navigation
    map's free space), so it crashed the whole demo instead of retrying. Now caught and
    logged as a retryable placement failure.
  - `MultiSim.stop_simulation()` called `synchronizer.stop()` (which clears
    `MultiSimSynchronizer._state_callback`) *before* `simulator.stop()` (which joins the
    simulation thread), so the simulation thread's `step()` could still be mid-read of
    that callback when it got cleared — `AttributeError:
    'NoneType' object has no attribute 'update_previous_world_state'`, silently
    swallowed since it's a background thread (`Exception in thread ...`, thread just
    dies) rather than crashing the process outright, but it killed that MuJoCo
    settling simulation and left later attempts unable to physically settle at all.
    Fixed by joining the simulation thread first.
  - **The big one**: `_make_shape_movable_in_mujoco` passed a shape's current pose as
    `Connection6DoF`'s `parent_T_connection_expression` (a fixed offset on top of the
    connection's own DOF values), but MuJoCo export only reads a `Connection6DoF`'s own
    DOF values into the free joint's keyframe `qpos` — which MuJoCo treats as the body's
    *absolute* world pose, not an offset relative to anything. Those DOFs default to
    identity, so **every settled shape started MuJoCo's gravity simulation at the world
    origin**, regardless of where it actually was. This was masked for whichever shape
    happened to be the *first* one ever settled in a `MontessoriWorld` — a one-time
    world-restructuring step in `MujocoBuilder.build_world` reparents that first shape's
    free joint via `World.move_branch`, which happens to write the DOF values directly —
    so it looked like things mostly worked until you looked at the second shape onward.
    Fixed by setting `connection.origin = current_pose` after creating the connection,
    using `Connection6DoF`'s own origin setter to write the DOF values directly instead
    of the offset MuJoCo export ignores. This is what limitation #1 below used to
    describe as a collision-decomposition problem — it mostly wasn't one.

## Known limitations (root cause understood, not fixed)

1. **Two of six shapes still don't fall through the holes in MuJoCo.** After fixing the
   world-origin pose bug above, a real end-to-end run (real HSRB, real Giskard planning,
   up to 3 insertion attempts per shape) gets **4 of 6 shapes falling through on their
   very first attempt** (`circular_hole_1_shape`, `square_hole_shape`,
   `triangle_hole_shape`, `rectangular_hole_shape`). The remaining two —
   `circular_hole_2_shape` and `disk_hole_shape` — still don't fall through after 3 real
   attempts each (confirmed each attempt actually reached and re-ran the MuJoCo settle,
   not blocked by `PointOccupiedError`). This is plausibly the genuine, previously-claimed
   collision-decomposition limitation: MuJoCo's default mesh collision uses the convex
   hull, which would erase the board's holes entirely, so the board's collision geometry
   is decomposed into ~40–50 convex pieces via CoACD (`_BOARD_COLLISION_DECOMPOSER` in
   `world.py`), imperfectly — a dropped shape can get caught on a residual,
   imperfectly-approximated lip rather than passing all the way through, and
   `disk_hole` (a narrow ~5mm slot) is the tightest-clearance hole on the board. This
   has *not* been re-confirmed via direct MuJoCo contact inspection since the pose fix
   (an earlier session did that inspection, but against data now known to be
   contaminated by the pose bug for every non-first shape) — worth re-verifying before
   trusting this framing further. Next step if revisited: re-run that contact inspection
   first; if it still holds, a hand-built cylindrical/slot collision insert per hole, or
   MuJoCo's SDF plugin (bundled but not wired into `semantic_digital_twin`'s MuJoCo
   adapter at all), are the next options.

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
   hole-opening precision. Not re-tested this session (still off).

## Running things

```
python3 -m pytest test/experiments_test/ -k montessori -v
```

From the repository root, against a real ROS 2 install (see above): full `test/`
subpackage-by-subpackage sweep (matching `ci_reusable.yml`'s per-package invocation,
`pytest -n auto test/<pkg>_test`) is green except:

- `test/robokudo_test/test_storage*.py` and `test_cr_descriptor_factory.py` need a real
  MongoDB server on `localhost:27017`; none is available by default (see the MongoDB
  note above for how to get one without breaking the ROS env).
- `test/physics_simulators_test/test_mujoco_simulator.py`'s rendering tests
  (`render_in_thread=True`, thousands of render steps) OOM (13GB+ RSS on a single
  process, no swap) or fail to init GLFW in a sandbox with no GPU/display — genuinely a
  sandbox limitation (`MUJOCO_LOG.TXT`: `ERROR: could not initialize GLFW`), not
  something these changes touched.

Everything else — `coraplex_test`, `semantic_digital_twin_test`, `giskardpy_test`,
`experiments_test` (including all 3 montessori-specific test files),
`robokudo_test` (once MongoDB is reachable), `krrood_test`, `random_events_test`,
`probabilistic_model_test` (with `QT_QPA_PLATFORM=offscreen`) — passes cleanly.

## Remaining open TODOs (not started)

From an earlier working-notes file (since removed):

- Give `montessori_demo.py` a configurable class for its main block instead of relying
  on module-level global constants.
- `world.py` could bind more of its scattered helper functions onto classes rather
  than leaving them as module-level functions.

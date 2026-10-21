# The HSR in RoboCasa

The Toyota HSR performs RoboCasa's pick-and-place and drawer tasks in a RoboCasa
kitchen, simulated in MuJoCo. RoboCasa composes the kitchen and samples the objects,
the HSR replaces RoboCasa's robot, every object is held by friction alone, and a
drawer moves only as far as the HSR's hold on its handle pulls or pushes it.

The kitchen is RoboCasa's layout 1, style 1, with everything standing on the floor
lowered from RoboCasa's 0.92 m counters to 0.75 m, where the HSR reaches.

| Task (class in `pick_place.py`) | What the HSR does | Seed | Object |
|---|---|---|---|
| `PickPlaceCounterToStove` | takes the food from the plate and puts it in the pan | 2 | cheese |
| `PickPlaceStoveToCounter` | takes the food from the pan and puts it in the bowl | 12 | lemon |
| `PickPlaceCounterToSink` | takes the object from the counter and drops it in the sink | 0 | can |
| `PickPlaceSinkToCounter` | takes the food out of the sink and puts it on the plate | 4 | yogurt |
| `PickPlaceDrawerToCounter` | pulls the drawer fully open, takes the object out and lays it on the counter past the stove | 9 | rolling pin |

| Task (class in `drawers.py`) | What the HSR does |
|---|---|
| `OpenDrawer` | takes the top drawer's handle from the front and pulls the drawer open |
| `CloseDrawer` | takes the handle of the drawer RoboCasa opened and pushes it shut |

| Task (class in `doors.py`) | What the HSR does |
|---|---|
| `OpenOven` | takes the bar along the top of the oven's door and lets the door down (layout 2, which has an oven) |
| `CloseOven` | takes the bar of the door RoboCasa let down from below and swings the door up, then takes the bar again from the front and presses the door shut (layout 2) |

The drawer tasks and `PickPlaceDrawerToCounter` lower the counters to 0.84 m only: RoboCasa takes a drawer for the
top drawer only while its middle stands 0.7 m to 0.9 m high. RoboCasa counts a drawer
open at 95 % of its travel and closed at 5 %, an oven door open at 90 % and closed at
0.5 %.

Every task is watched by SegMind while it runs, and the tests take its word for
success: a pick-and-place task has to end with a placing of the object on its
receptacle, a drawer or door task with the drawer or door seen opened or closed at
RoboCasa's thresholds.

## One-time setup

1. **robosuite and RoboCasa**, installed editable into the project's venv without
   their dependencies, so they leave MuJoCo and numpy as the project pins them:

   ```bash
   cd ~/cram2
   git clone https://github.com/ARISE-Initiative/robosuite.git
   git clone https://github.com/robocasa/robocasa.git
   cd ~/cram2/cognitive_robot_abstract_machine
   .venv/bin/python -m pip install --no-deps -e ~/cram2/robosuite -e ~/cram2/robocasa
   .venv/bin/python -m pip install termcolor h5py gymnasium
   ```

   The last line adds the few packages robosuite needs at import.

2. **RoboCasa's assets** (kitchen fixtures and objects, about 6 GB):

   ```bash
   .venv/bin/python -m robocasa.scripts.download_kitchen_assets
   ```

   The loader reads them from `~/robocasa-assets`; if the script put them elsewhere,
   link that folder there.

3. **A one-line fix in RoboCasa.** RoboCasa removes duplicate counter regions with
   `list(set(...))` over XML elements, which hash by memory address, so the same seed
   puts objects in different places in different runs. Keep the order instead:

   ```bash
   sed -i 's/valid_geoms = list(set(valid_geoms))/valid_geoms = list(dict.fromkeys(valid_geoms))/' \
       ~/cram2/robocasa/robocasa/models/fixtures/counter.py
   ```

   Without it, a task can sample a different scene than its seed promises, and its
   full run can fail.

## Every shell

ROS provides `rclpy`, and `~/ros_ws` provides the HSR's description
(`hsr_description`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_ws/install/setup.bash
cd ~/cram2/cognitive_robot_abstract_machine
```

## Running the tests

**Fast tests** (the loader, the lowered layout, the HSR's arm), about 45 s:

```bash
.venv/bin/python -m pytest --orm-build never -q \
    test/semantic_digital_twin_test/test_robocasa \
    test/experiments_test/hsr_robocasa_test
```

The nine full runs show as skipped: they only run with `CI=true`.

**Full runs**: each task performed in MuJoCo, about 90 s per pick-and-place task and
60 s per drawer task. Cap the memory,
so that a runaway run is killed rather than the machine:

```bash
CI=true systemd-run --user --scope -q -p MemoryMax=14G \
    .venv/bin/python -m pytest --orm-build never -q test/experiments_test/hsr_robocasa_test
```

One task only:

```bash
CI=true .venv/bin/python -m pytest --orm-build never -q \
    test/experiments_test/hsr_robocasa_test -k PickPlaceCounterToSink
CI=true .venv/bin/python -m pytest --orm-build never -q \
    test/experiments_test/hsr_robocasa_test -k opens_the_drawer
```

Run the tests one after the other, never two at the same time: every RoboCasa scene
is written to the same `/tmp/scene.xml`, so parallel runs overwrite each other's
scene.

If a test fails on an ORM import, regenerate the ORM interfaces:

```bash
.venv/bin/python scripts/regenerate_all_orm.py
```

## Watching a task

With MuJoCo's viewer, the stove task:

```bash
.venv/bin/python -m experiments.hsr_robocasa.pick_place
```

The drawer opening task:

```bash
.venv/bin/python -m experiments.hsr_robocasa.drawers
```

Any other task:

```bash
.venv/bin/python -c "
from experiments.hsr_robocasa.pick_place import PickPlaceSinkToCounter
task = PickPlaceSinkToCounter()
task.build_world()
task.run()
"
```

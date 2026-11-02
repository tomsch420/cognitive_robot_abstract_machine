# RoboCasa's atomic tasks, performed by a PR2

A PR2 performs every one of RoboCasa's atomic kitchen tasks, in a kinematic simulation
with collision avoidance. RoboCasa composes the kitchen, samples the objects and says
where its own robot starts; the PR2 stands there, does the task with coraplex, and
RoboCasa's own success check judges it.

There is one module per file RoboCasa defines its atomic tasks in, and one class per
task in it, named after the RoboCasa task:

| Module | RoboCasa file | Tasks |
|---|---|---|
| `blender.py` | `kitchen_blender.py` | `OpenBlenderLid`, `CloseBlenderLid`, `TurnOnBlender` |
| `coffee.py` | `kitchen_coffee.py` | `CoffeeSetupMug`, `CoffeeServeMug`, `StartCoffeeMachine` |
| `doors.py` | `kitchen_doors.py` | `OpenCabinet`, `CloseCabinet`, `OpenMicrowave`, `CloseMicrowave`, `OpenFridge`, `CloseFridge`, `OpenOven`, `CloseOven`, `OpenDishwasher`, `CloseDishwasher`, `OpenToasterOvenDoor`, `CloseToasterOvenDoor` |
| `drawer.py` | `kitchen_drawer.py` | `OpenDrawer`, `CloseDrawer`, `SlideDishwasherRack`, `OpenFridgeDrawer`, `CloseFridgeDrawer` |
| `electric_kettle.py` | `kitchen_electric_kettle.py` | `TurnOnElectricKettle`, `CloseElectricKettleLid`, `OpenElectricKettleLid` |
| `microwave.py` | `kitchen_microwave.py` | `TurnOnMicrowave`, `TurnOffMicrowave` |
| `navigate.py` | `kitchen_navigate.py` | `NavigateKitchen` |
| `oven.py` | `kitchen_oven.py` | `PreheatOven`, `SlideOvenRack` |
| `pick_place.py` | `kitchen_pick_place.py` | the 18 `PickPlace...` tasks, `CheesyBread`, `MakeIcedCoffee`, `PackDessert` |
| `sink.py` | `kitchen_sink.py` | `TurnOnSinkFaucet`, `TurnOffSinkFaucet`, `TurnSinkSpout`, `AdjustWaterTemperature` |
| `stand_mixer.py` | `kitchen_stand_mixer.py` | `OpenStandMixerHead`, `CloseStandMixerHead` |
| `stove.py` | `kitchen_stove.py` | `TurnOnStove`, `TurnOffStove`, `LowerHeat` |
| `toaster.py` | `kitchen_toaster.py` | `TurnOnToaster` |
| `toaster_oven.py` | `kitchen_toaster_oven.py` | `AdjustToasterOvenTemperature`, `TurnOnToasterOven`, `SlideToasterOvenRack` |

RoboCasa's own abstract bases (`ManipulateDoor`, `PickPlace`, ...) are left out: they
are parametrised by the tasks above rather than tasks of their own. The oven and stove
tasks run in layout 2: layout 1 has no oven, and its range has its knobs on a panel at
the back, beyond the reach of a PR2 standing clear of it; layout 2's cooktop has them
along its front edge.

`episode.py` holds what every task shares, and `skills.py` the actions coraplex does
not ship: moving a fixture's joint by a part hanging from it, pressing and pushing with
the fingertips, and standing somewhere picked from a list of candidates.

## Running

One task:

```bash
python -m experiments.robocasa_atomic_tasks.drawer OpenDrawer
python -m experiments.robocasa_atomic_tasks.pick_place PickPlaceCounterToSink --seed 3
python -m experiments.robocasa_atomic_tasks.stove TurnOnStove --layout LAYOUT002
```

Every task, each in a process of its own, with a table of which succeeded, and with
`--videos` a video of each seen from the PR2's head camera:

```bash
python -m experiments.robocasa_atomic_tasks --workers 8 --results ~/atomic_tasks
python -m experiments.robocasa_atomic_tasks --workers 8 --videos --results ~/atomic_tasks
python -m experiments.robocasa_atomic_tasks.drawer OpenDrawer --video open_drawer.mp4
```

A video is recorded without MuJoCo while the task runs (`recording.py`): the poses of
the bodies that move are kept at every change of the world's state, and afterwards a
forked process replays them in a MuJoCo scene of the kitchen and renders them from the
PR2's wide stereo camera, turned towards the gripper as the head would be.

RoboCasa and robosuite have to be importable and RoboCasa's kitchen assets downloaded,
as for the other RoboCasa experiments.

## How the tasks are judged

RoboCasa's simulation is never stepped; it only judges. After the PR2 has acted, the
poses of the objects, of loose fixture parts such as the blender's lid, and the
positions of the fixture joints are copied into it, every fixture updates its state
from them as RoboCasa does after each of its steps, and the frames of RoboCasa's own
robot that its checks read (its gripper's end effector site and its base) are put where
the PR2's are. Then RoboCasa's `_check_success` decides.

What a kinematic simulation lacks is stood in for:

- **Contact.** The microwave's, the coffee machine's and the blender's buttons are
  sensed by RoboCasa through contact with its gripper. While the PR2's fingertips rest
  on such a button, RoboCasa's contact check is told the gripper touches it.
- **Gravity.** An object is let go of just above its spot and then lowered until it
  touches what is below, both in the world and in RoboCasa's own model, whose collision
  geometry differs, so that RoboCasa finds the two in contact. An object for the sink is
  let go of above the rim and drops onto the basin's floor that way.
- **Detents.** The motion controller stops a pushed joint a few tenths of a millimetre
  short of its end stop, most of the 4 mm travel of the kettle's switch; a switch or
  lever pushed within a tenth of its travel of the end snaps there.

Where RoboCasa's kettle opens its lid by itself once its button is pressed, it does so
only as simulated time passes; the PR2 opens the lid by hand instead.

## How the PR2 does them

- **Doors, drawers, racks, knobs, levers, lids, faucet handles:** the PR2 takes hold of
  the part hanging from the joint (its handle, where it has one) and drives the joint to
  where RoboCasa counts the task done, holding the part all the way.
- **Buttons and switches:** the PR2 closes its gripper and pushes its fingertips onto
  the button, and on along the switch's travel where the switch is a joint.
- **Pick and place:** the PR2 picks the object up and puts it down inside the fixture
  as RoboCasa bounds its inside, on a spot of a counter's top RoboCasa places objects
  on, or on top of the receptacle.
- **Navigation:** the PR2 drives along a path through the free space of the kitchen's
  floor to where RoboCasa would stand its own robot, or as near as it fits.

The PR2 stands where it can reach what it acts on: in front of it, facing the kitchen
the way RoboCasa's start pose does, a little to the left so that it lies ahead of the
right shoulder, and clear of the kitchen. Its base takes part in every motion of the
arm, and its torso is raised for anything at a counter's height or above. The grasps
tried come in along the direction the robot faces the kitchen, from above, or rising
from below for a wall cabinet's handle; objects are taken from above first. coraplex
tries the stands and the grasps until one works.

While the PR2 acts on a fixture, or picks an object up or puts it down, its gripper may
touch that fixture and those right around it, and the rest of the arm may come within a
centimetre of them; everything else is kept at the distances collision avoidance
always keeps. Without this the gripper is held back by the very fixture a knob sits on,
and the forearm by the counter it reaches across.

## Running into trouble

coraplex discards a failed candidate without saying why. To see why every stand and
grasp failed, run a task through a wrapper that logs the `PlanFailure` of each trial and
the robot's links inside their collision buffer at that moment (see
`ActionTrial.succeeds`).

## Where it stands

A run of every task with seed 0 accomplished 30 of the 65 before paths through free
space were added, and 29 with them as a fallback: `OpenOven` was solved only with the
free-space path tried first, and `TurnOnStove` and `PickPlaceCounterToStandMixer` fail
in some runs, where a candidate that passed coraplex's trial in a copy of the world then
violates collision avoidance when performed. The run before free-space paths:

- **Accomplished:** `AdjustToasterOvenTemperature`, `CloseDishwasher`, `CloseDrawer`,
  `CloseElectricKettleLid`, `CloseFridge`, `CloseFridgeDrawer`, `CloseMicrowave`,
  `CloseOven`, `CloseStandMixerHead`, `CoffeeServeMug`, `LowerHeat`, `MakeIcedCoffee`,
  `OpenDrawer`, `OpenElectricKettleLid`, `OpenFridgeDrawer`, `OpenMicrowave`,
  `OpenStandMixerHead`, `PickPlaceCounterToOven`, `PickPlaceCounterToSink`,
  `PickPlaceCounterToStandMixer`, `PickPlaceCounterToStove`, `PickPlaceSinkToCounter`,
  `PreheatOven`, `SlideToasterOvenRack`, `TurnOffMicrowave`, `TurnOnBlender`,
  `TurnOnElectricKettle`, `TurnOnMicrowave`, `TurnOnToasterOven`, `TurnSinkSpout`.
- **Accomplished in some runs only:** `TurnOnStove` (in layouts 2, 7 and 9, not in
  every run); the faucet's handle is reachable on its own, but `TurnOnSinkFaucet`,
  `TurnOffSinkFaucet` and `AdjustWaterTemperature` fail in full runs.
- **Out of the PR2's reach:** the wall cabinets' handles sit 1.58 m high above a 0.65 m
  deep counter in every layout; reaching them, the forearm runs into the cabinet's
  bottom (`OpenCabinet`, `CloseCabinet`, `PickPlaceCounterToCabinet`,
  `PickPlaceCabinetToCounter`).
- **Not solved yet:** the drop-down doors and racks of the dishwasher, the oven and the
  toaster oven (whose door has no handle of its own), `OpenFridge`, `TurnOffStove`,
  `TurnOnToaster`, `StartCoffeeMachine`, `CoffeeSetupMug`, the blender's lid, the
  remaining pick and place tasks, mostly into the microwave, the fridge and the toaster
  oven and onto the counter, and `NavigateKitchen`, where the PR2, whose parked arms
  span 1.57 m, sweeps them through the fixtures it drives past.

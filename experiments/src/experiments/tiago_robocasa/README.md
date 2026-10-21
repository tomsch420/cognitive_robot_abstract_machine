# Tiago in RoboCasa

Tiago performs RoboCasa's pick-and-place tasks in a RoboCasa kitchen, simulated in
MuJoCo. RoboCasa composes the kitchen and samples the objects, Tiago replaces RoboCasa's
own robot, and every object it moves is held by the grip of its fingers alone: nothing
attaches an object to the hand.

The kitchen is **not** scaled down. RoboCasa's counters stand at 0.92 m and Tiago reaches
them, so it works in the kitchen as it ships, the way the PR2 does.

| Task | What Tiago does |
|---|---|
| `PickPlaceCounterToSink` | takes the object off the counter; with `--pick-only`, stops once it is clear of the counter |
| `PickPlaceCounterToStove` | takes the food off the plate and puts it in the pan |

## Running one

The experiment composes kitchens until it finds one it can work in, because RoboCasa
samples a different object and placement every time, even for the same seed:

```bash
python -m experiments.tiago_robocasa.pick_place PickPlaceCounterToSink \
    --pick-only --attempts 12 --record
```

`--benchmark` attempts every episode instead of stopping at the first that works, and
reports how many ended with the object off its support. `--record` writes a video of the
first episode that works, by default into `~/Downloads`; it draws through the graphics
driver, so a machine whose driver does not match its kernel module needs
`__GLX_VENDOR_LIBRARY_NAME=mesa`.

## What Tiago needed that the other robots did not

- **An armature on every servoed joint.** Tiago's MuJoCo description states none, so a
  wrist held by a stiff servo has only the links beyond it for inertia. Within a second
  of simulated time such a joint oscillates apart and takes the arm to hundreds of
  thousands of radians. `TiagoMujocoServo.armature` states the inertia of what spins
  behind the joint.
- **MuJoCo's semi-implicit integrator and its conjugate-gradient solver.** The default
  Newton solver factorizes a Hessian this kitchen makes rank-deficient once the hand is
  among the fixtures, and the simulation stops dead.
- **Goals held to what a simulated arm can hold.** The arm trails what it is asked for by
  about a tenth of a radian while it moves, so goals held to millimetres are never
  reached and the motion gives up with the hand short of the object.
- **A stand that is looked for rather than computed.** Tiago is wider than the robot a
  RoboCasa kitchen is laid out around and carries a second arm at its side, so the stand
  that puts its working hand in front of the object often leaves a wheel in a cabinet or
  an elbow in a wall. Stands are tried back and along the counter until one clears the
  kitchen *and* lets the arm reach.
- **The arm commanded in joint space.** Where the arm is to go is worked out in a
  kinematic world and commanded as joints: a tool frame goal given to the physical run is
  reported reached while the arm is still on its way.
- **A grasp from above.** Coming down onto the object keeps the wrist over the worktop;
  reaching in level with the counter swings it through the cabinets below.

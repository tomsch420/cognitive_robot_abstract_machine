PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/752 (draft), branch pr2-mujoco-physics on tomsch420, based on cram2/main be2ac4f93.

Plan: make the PR2 physically simulatable in MuJoCo, driven by giskard.
- Done: PR2 servos for arms, torso, head and the gripper thumbs; drive servos on x, y, yaw built as joints pr2_omni_drive_x/y/yaw; the world to simulation sync commands every actuated DOF; the pacer waits for the base.
- Done: body masses in MuJoCo come from the inertial (inertiafromgeom auto in the builder and in MujocoSimulator); Body.inertial defaults to None (user chose this); minimum mass and inertia for empty frames; balanceinertia; servos on unlimited joints left unclamped.
- Done: tests on neem-4 with CI=true: the twin, physics_simulators and confidence EQL 2,214 passed; giskardpy 1,149 passed; coraplex subsets 456 passed.
- Known, not caused by this PR: Tracy's parked arms test and its live control test fail on main too.

Next: wait for review. Not done yet: the PR2 base does not touch the floor (no floor contact tested); the RoboCasa counter to sink task in physics (robocasa-experiments branch) builds on this.

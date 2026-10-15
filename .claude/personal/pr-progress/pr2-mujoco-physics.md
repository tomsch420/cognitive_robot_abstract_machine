PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/752 (draft), branch pr2-mujoco-physics on tomsch420, based on cram2/main be2ac4f93.

Plan: make the PR2 physically simulatable in MuJoCo, driven by giskard.
- Done: PR2 servos for arms, torso, head and the gripper thumbs; drive servos on x, y, yaw built as joints pr2_omni_drive_x/y/yaw; the world to simulation sync commands every actuated DOF; the pacer waits for the base.
- Done: body masses in MuJoCo come from the inertial (inertiafromgeom auto in the builder and in MujocoSimulator); Body.inertial defaults to None (user chose this); minimum mass and inertia for empty frames; balanceinertia; servos on unlimited joints left unclamped.
- Done: tests on neem-4 with CI=true: the twin, physics_simulators and confidence EQL 2,214 passed; giskardpy 1,149 passed; coraplex subsets 456 passed.
- Done: the description shows a GIF and links an MP4 of the PR2 driving; both live on the fork branch pr2-mujoco-physics-media, not on the PR branch.
- Done (commit 036fe2150): CI failed on Tracy's parked arms test. Cause: MujocoBuilder cached Collada conversions as <asset folder>/<stem>.stl and named mesh assets by stem, so Tracy reused the PR2's base and forearm meshes once the new PR2 tests had built the PR2. Derived files now mirror the source path and every file and scale gets its own asset. The earlier note that the Tracy tests fail on main was wrong; that was the same stale cache in the neem-4 container. semantic_digital_twin 2,167 passed, physics_simulators 25 passed.

Next: wait for review. Not done yet: the PR2 base does not touch the floor (no floor contact tested); the RoboCasa counter to sink task in physics (robocasa-experiments branch) builds on this.

PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/1067 (draft, upstream, label new-feature); fork PR https://github.com/tomsch420/cognitive_robot_abstract_machine/pull/57

Plan:
- Opt in scaling of each external contact's buffer zone by the translational distance from its closest point on the robot to a goal point

Done:
- BufferZoneScaling (abstract, per contact point) and LinearBufferZoneScaling, field buffer_zone_scaling on ExternalCollisionAvoidance; tip_link removed
- Tests for factor bounds, per contact factors, violated distance floor, full motion (also via JSON); 324 tests pass; PR descriptions updated

Next:
- CI fixed (krrood check --since docstrings, a72b49be97); end to end ROS test added (5042a1f34a); confirm CI on 1067 is green
- Await review on cram2 PR 1067

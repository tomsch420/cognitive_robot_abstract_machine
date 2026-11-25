PR: https://github.com/cram2/cognitive_robot_abstract_machine/pull/1067 (draft, upstream, label new-feature); fork PR https://github.com/tomsch420/cognitive_robot_abstract_machine/pull/57

Plan:
- Opt in scaling of external collision buffer zones by the translational distance from a tip link to a goal point

Done:
- BufferZoneScaling (abstract) and LinearBufferZoneScaling, field buffer_zone_scaling on ExternalCollisionAvoidance
- Tests for factor bounds, constraint lower error, violated distance floor, full motion (also via JSON); 324 tests pass

Next:
- Await review on cram2 PR 1067

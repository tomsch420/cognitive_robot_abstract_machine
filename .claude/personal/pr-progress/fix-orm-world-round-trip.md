## PR cram2#1032 (fix-orm-world-round-trip): store inertia tensors, camera bodies and actuator degrees of freedom through ormatic (fixes cram2#1008)
- Plan: NPMatrix3x3/NPVector3 data as a required np.ndarray (ormatic maps it), MultiSimCamera.body typed Body | None, AddActuatorModification keeps degree_of_freedom_ids; round trip test per loss
- Done: commits up to the one requiring np.ndarray data on cram2#1032 (draft, labels bug-fix, api-break); list[float] variant reverted; PR description updated
- Next: CI of cram2#1032; discuss the api-break with semantic_digital_twin code owners; maybe krrood issues: ormatic does not map the NDArray alias nor optionals like list[float] | None

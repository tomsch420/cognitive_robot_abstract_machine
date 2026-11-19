## PR cram2#1032 (fix-orm-world-round-trip): store inertia tensors, camera bodies and actuator degrees of freedom through ormatic (fixes cram2#1008)
- Plan: NPMatrix3x3/NPVector3 data as a required NDArray; krrood TypeDict looks up parametrized generics by origin so NDArray maps to the numpy array column; MultiSimCamera.body typed Body | None; AddActuatorModification keeps degree_of_freedom_ids
- Done: last commit b2d6c0a87e on cram2#1032 (draft, labels bug-fix, api-break, approved by a reviewer); review thread on TypeDict abbreviations answered and resolved
- Next: CI of cram2#1032; discuss the api-break with semantic_digital_twin and krrood code owners; maybe a krrood issue for optionals like list[float] | None not being stored

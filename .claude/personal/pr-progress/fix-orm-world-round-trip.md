## PR cram2#1032 (fix-orm-world-round-trip): store inertia tensors, camera bodies and actuator degrees of freedom through ormatic (fixes cram2#1008)
- Plan: InertiaTensor.data as list[float] (nine elements, matrix property), MultiSimCamera.body typed Body | None, AddActuatorModification keeps degree_of_freedom_ids; round trip test per loss
- Done: commits 08e1dc2a03, de4943f6aa (InertiaTensorMapping replaced by list[float] data) on cram2#1032 (draft, labels bug-fix, api-break); new tests fail on main and pass here
- Next: CI of cram2#1032; discuss the api-break with semantic_digital_twin code owners; maybe a krrood issue for `list[float] | None` mapping fields not being stored

<!-- BEGIN-PR-PROGRESS -->
## PR progress: tensorized-circuit-orm (cram2#1083, draft)

### Plan
- Store layered probabilistic circuits with ORMatic, no alternative mapping

### Done
- BackendArray class + BackendArrayType; Layer as krrood Interface (ffb8e0f7af); SortedSet[Variable] variables
- ORMatic SortedSet relationships (rebuilt after post init); virtual subclass path length fix
- probabilistic_model ORM interface, generated from the whole package like the other generators (f8a213d09b)
- Interface docstring: declares what subclasses provide, not how it is stored (4dca6217e1, 7e6b115eaa)
- Review round 1 settled: all 3 threads resolved (subclasshook keeps NotImplemented)
- Review round 2: SubclassJSONSerializer as ORMatic default ignored class (aefa0ba6c5); alias names resolved in annotated modules so TrainingState is mapped (2508f010dc); 388 DAOs in pm
- Full mapping (b15b34a820): BackendArray takes __array_namespace__ classes (jax.Array), BackendArrayJSONSerializer; jax/torch modules only mapped where installed (366 DAOs without them, verified)
- Review round 3: generic docstrings and flat loops in alias resolution (786a82e580); external serializers ignored by default; coverage 386 of 413 pm classes (93.5 %), all data dataclasses
- optimizer_state skipped by agreement; noted in thread and description (thread resolved)
- Tests green on neem-4; LVD ad circuit round trips at 5.75 MB vs 13 GB JSON

### Next
- CI on cram2#1083 at the latest push
- Possible follow ups: circuit learning runner stores circuits through the ORM; optimizer state as leaves rebuilt from optimizer.init
<!-- END-PR-PROGRESS -->

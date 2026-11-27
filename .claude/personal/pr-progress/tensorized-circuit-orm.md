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
- Tests green on neem-4; LVD ad circuit round trips at 5.75 MB vs 13 GB JSON

### Next
- CI on cram2#1083 (24 checks pending at 7e6b115eaa)
- Possible follow up: circuit learning runner stores circuits through the ORM instead of JSON
<!-- END-PR-PROGRESS -->

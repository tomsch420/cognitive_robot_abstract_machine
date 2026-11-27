<!-- BEGIN-PR-PROGRESS -->
## PR progress: tensorized-circuit-orm (cram2#1083, draft)

### Plan
- Store layered probabilistic circuits with ORMatic, no alternative mapping

### Done
- BackendArray class + BackendArrayType, Layer dataclass, SortedSet[Variable] variables
- ORMatic SortedSet relationships (rebuilt after post init), virtual subclass path length fix
- probabilistic_model ORM interface and generator, registered in orm_interfaces.py
- Tests; suites green on neem-4 (tensorized-orm tree); LVD ad circuit: 3.82 MB vs 13 GB JSON, identical log likelihoods
- Rebased onto cram2/main 913a92c231, pushed, draft PR opened (new-feature, bug-fix)

### Next
- Wait for CI and review on cram2#1083
- Possible follow up: circuit learning runner stores circuits through the ORM instead of JSON
<!-- END-PR-PROGRESS -->

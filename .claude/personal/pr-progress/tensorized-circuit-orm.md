<!-- BEGIN-PR-PROGRESS -->
## PR progress: tensorized-circuit-orm (cram2#1083, draft)

### Plan
- Store probabilistic models with ORMatic, no alternative mapping

### Done
- probabilistic_model interface over the whole package: 386 of 413 classes (93.5 %), every data dataclass
- Merged cram2/main 564a51f56a (4b60f45e8f); ORMatic collections now via upstream CollectionShape, SortedSet restored after post init
- Region renamed to VariableRegion (7632d32203); annotated module lookup moved onto WrappedClass (db5fe7e4d3)
- Review threads resolved except the Region one; labels new-feature, bug-fix, api-break; migration notes in the description

### Next
- Answer pending: should semantic_digital_twin's interface build on probabilistic_model's (pm and sdt interfaces both declare ExceptionMappingDAO, FunctionMappingDAO, so they cannot be imported together)?
- Discuss the api-break (frozenset fields now stored) with code owners: cramph @ichumuh @Tigul, giskardpy @ichumuh @maltehue, coraplex @Tigul @sunava
- Check the coraplex ORM tests in CI (not runnable on neem-4: need iai_objects)
- Follow ups: circuit learning runner stores circuits through the ORM; optimizer state as leaves rebuilt from optimizer.init
<!-- END-PR-PROGRESS -->

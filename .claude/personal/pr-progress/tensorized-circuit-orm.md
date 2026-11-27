<!-- BEGIN-PR-PROGRESS -->
## PR progress: tensorized-circuit-orm (cram2#1083, draft)

### Plan
- Store probabilistic models with ORMatic, no alternative mapping

### Done
- probabilistic_model interface over the whole package: 386 of 413 classes (93.5 %), every data dataclass
- Merged cram2/main 564a51f56a (4b60f45e8f); ORMatic collections now via upstream CollectionShape, SortedSet restored after post init
- All review threads resolved; labels new-feature, bug-fix, api-break; migration notes in the description

### Next
- Discuss the api-break (frozenset fields now stored) with code owners: cramph @ichumuh @Tigul, giskardpy @ichumuh @maltehue, coraplex @Tigul @sunava
- Check the coraplex ORM tests in CI (not runnable on neem-4: need iai_objects)
- Follow ups: circuit learning runner stores circuits through the ORM; optimizer state as leaves rebuilt from optimizer.init
<!-- END-PR-PROGRESS -->

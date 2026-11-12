## PR #945 (inner-layers-any-backend)
- Plan: inner layers on any backend through an abstract EdgeMatrix (sparse, dense); array_types split; plan items updated
- Done: EdgeMatrix rework (45e3d0c953), array_types split and to_coo_array removed (6ffdee09b4); 1571 tests pass; benchmark: RAT-SPN inference +2 %, truncation -10 %, conditioning -33 %; later plan items and the indexing standing rule saved
- Next: review

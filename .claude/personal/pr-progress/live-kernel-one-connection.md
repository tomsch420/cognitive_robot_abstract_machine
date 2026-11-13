## PR #955 (live-kernel-one-connection, worktree /home/tom_sch/cram-live-session)
- Plan: cut the live session's per step transport time in live_kernel.py (shared ssh connection, one call to start a run, streamed output)
- Done: implemented and measured on neem-4 (empty cell 7 s -> 1.1 s), draft PR opened; review threads (no abbreviations) answered and resolved in 8bea9fb696; related viewer slowdown is AICOR-Solutions/cramera#121, patched in both live kernels
- Next: wait for further review; ask whether to add tests for the script

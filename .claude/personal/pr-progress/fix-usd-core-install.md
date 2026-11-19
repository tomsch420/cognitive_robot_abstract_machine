## PR cram2#1021 (fix-usd-core-install): Install usd-core in CI for semantic_digital_twin
- Plan: the *Install usd-core* step of ci_reusable.yml tests the undeclared inputs.lib (cram2#996); test inputs.libs like the drake step and add a test that every workflow reads only declared inputs
- Done: commit 3ae6f1bdb7 on cram2#1021 (draft, label bug-fix); the new test failed before the fix and passes after it (run with --noconftest, since the local venv lacks krrood_compute_resources)
- Next: CI of cram2#1021, where the semantic_digital_twin USD tests run for the first time and may fail; a `bug` label does not exist upstream

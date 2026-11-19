## PR cram2#1021 (fix-usd-core-install): Install usd-core in CI for semantic_digital_twin, and allow opencv-python 5.1
- Plan: the usd-core step of ci_reusable.yml tests inputs.libs instead of the undeclared inputs.lib (cram2#996); drop robokudo's opencv-python<5.1 bound, since 5.1.0.95 now has Linux wheels. The developer wants no python test for the workflow fix
- Done: commits 8ac8961966 and 4b757ceab4 on cram2#1021 (draft, labels bug-fix, maintenance)
- Next: CI of cram2#1021, where the semantic_digital_twin USD tests and a fresh opencv-python 5.1 resolution run for the first time

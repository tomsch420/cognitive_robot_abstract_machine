## PR cram2#961 (torch-backend)
- Plan: Run and train tensorized circuits on torch, on cram2 as a draft after #953 merged
- Done: projection fix carried from #953; loss as a field instead of empty subclasses (659920ec32, api-break); second review answered (3851fb7299: GradientDescent docstring names TorchGradientDescent, torch normal functions keep ArrayType unbound, ArrayBackend StrEnum in the tests, minimum_improvement fixture); all threads replied and resolved; stack below merged forward and passing
- Next: re-review; api-break discussion with the code owners

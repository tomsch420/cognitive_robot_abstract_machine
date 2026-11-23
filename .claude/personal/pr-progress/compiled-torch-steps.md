### Plan
- Opt-in `compiles_steps` on the torch learners: trace the step's computation (loss/log-likelihood, gradients, EM statistics) with make_fx and compile it with torch.compile per batch shape; on in the benchmark's torch learners.

### Done
- Implemented `compute_on_batch` / `computation_on_batch` / `CompiledComputation` (9db1f09ee1), tests in `test_compiled_torch_learning.py` and `test_methods.py`; 1,019 tests pass, `krrood check` clean.
- Measured on HEPMASS f32: GPU EM 0.93 -> 0.355 s/epoch, GPU GD 0.88 -> 0.375, CPU EM 16.2 -> 10.5; losses identical; compiling 8-27 s in epoch 1.
- Merged #55's CUDA-capture fix (394081c82c, cause was GC freeing an old training's graph mid-capture, not inductor threads); 116 GPU tests pass; description updated.
- Draft PR #56 stacked on #55; label `new-feature` named to the user (missing on the fork).

### Next
- Nothing open; wait for review or further instructions.

## cram2#1046 Train block-edge circuits up to 10x faster with jax and 4x with torch (branch block-edges-without-gathers)

**Plan**
- All measured training optimizations in one PR: gather-free block reduction, sums as matrix products, products as cross products, gradient stop on log-sum-exp shifts, one loss sync per epoch, torch steps recorded as CUDA graphs.

**Done**
- 1055c0eb40 pushed: jax CPU batches split across host devices (runner: largest power of two of share cores); 25-core share 33.2 -> 6.54 s/epoch vs cirkit 40.1; torch GPU 14.5 -> 3.84 ms with CUDA graphs (harness 1.22 s/epoch vs cirkit 6.41); jax GPU unchanged; suite 1869 passed, 1 xfailed.
- bc5733cb2e pushed: review comments addressed (mesh -> batch_split_devices/BatchSplitAxis; BatchShape, DevicePlacement and StructureArrayCopy replace tuple keys); threads replied to and resolved.
- Description has the final 100-epoch comparison with cirkit: jax beats it everywhere (GPU 0.39/1.57 vs 6.41/6.15, 25-core CPU 8.8/13.6 vs 22.0/37.7 s/epoch); our torch CPU f64 49.2 is slower than cirkit's 37.7.

**Next**
- Developer discusses the api-break with the probabilistic_model code owners.
- Open: torch on the CPU in float64 is slower than cirkit (no graph recording there). GradientStop and NormalDistributionFunctions each pick their backend class (sharing a base needs the monkeypatch test changed); CUDA graphs are opt-in until truncated Gaussian, binomial and probability-table layers stop copying from the host.

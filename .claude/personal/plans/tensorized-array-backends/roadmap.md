# Tensorized circuits on any array backend — standing rules

- Backend neutral code uses the Python Array API standard through
  array-api-compat (xp = array_namespace(x)) and array-api-extra; never write our
  own backend interface where these suffice.
- Gathers index with numpy integer arrays (values[..., index]) instead of xp.take,
  which was 32 % slower on numpy; numpy, jax and torch support such indexing,
  although the Array API standard does not require it.
- Structure is numpy on the host and fixed at construction; only parameters and
  query values are backend arrays. Never store a backend array in a static field.
- Numeric query paths have no branches on array values, no data dependent shapes,
  no in place writes and no caches of traced values, so they run under jax.jit.
- Structural queries (support, marginal, truncation, simplify, mode, determinism)
  and sampling stay numpy only on the host.
- Every numeric query test runs on numpy and jax (float64 enabled), and on torch
  once the torch backend exists.
- Each item is its own PR against cram2 main, opened from the fork as a draft.
- Measure performance changes with /root/ratspn_vs_jpt/evaluate.py in
  tom-probabilistic-experiments on neem-4 before and after.

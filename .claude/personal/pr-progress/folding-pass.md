## PR cram2#1033 (folding-pass): fold the layers of layered circuits, train Gaussian scales through their logarithm
- Plan: folding stack (#51-#54) plus Gaussian log scale; benchmark measures folded circuits with block edges
- Done: head 8cd3eae023 (draft): review threads resolved; main merged; placed folds, EdgeRepresentation.BLOCK, block reference, JPT placed
- Next: full benchmark against cirkit on neem-4 (/root/circuit-learning-results/bench-full, scratch branch bench-full); cirkit-comparison #47 needs fold=False and log_scale when merged after this

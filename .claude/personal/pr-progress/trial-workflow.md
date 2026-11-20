## PR cram2#1040 (trial-workflow): find out by trials what makes a behaviour succeed
- Plan: A Trial[P,R] + TrialResult with mediators; B DatabaseTrialStore through ormatic, read back by starting state; C pilot trials refuse an outcome that cannot vary; D repetitions; E memory budget + clean stop; F StartingStateRecipe on a shared worker pool; G circuit analysis, best region, Mann Whitney (scipy declared in krrood-probabilistic), causal query; H next_registry + adopted defaults
- Done: plan agreed with the user; baseline 16/16 trial tests green in tom-cram-parametrized-trials at efb44e4722
- Next: A

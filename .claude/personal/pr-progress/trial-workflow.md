## PR cram2#1040 (trial-workflow): find out by trials what makes a behaviour succeed
- Plan: A Trial[P,R] + TrialResult with mediators; B DatabaseTrialStore through ormatic, read back by starting state; C pilot trials refuse an outcome that cannot vary; D repetitions; E memory budget + clean stop; F StartingStateRecipe on a shared worker pool; G circuit analysis, best region, Mann Whitney (scipy declared in krrood-probabilistic), causal query; H next_registry + adopted defaults
- Done: A 9cddcc6ed8, B 7edccbc308, C 86bf344ee6, D 4b218cc156; trial tests green in tom-cram-parametrized-trials (files synced from the worktree; the neem-4 clone has cram2 as origin, so nothing is committed there)
- Next: E memory budget and clean stop, then F, G, H; PR description update at the end

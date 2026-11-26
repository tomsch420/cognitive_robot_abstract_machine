Plan
- Benchmark HCLT against RAT-SPN and JPT on neem-4 and put the tables in the PR description
- Developer confirms or corrects the prototype defaults (12 hidden states, 10 bins, init [0.1, 1], mini-batch 512 rows / step 0.1)

Done
- Library, benchmark method, review round 1, selectable mutual information estimator, discretize_rows rename (latest 0385ebfbe0), merged cram2 main (9044427246) after #1070 fixed main's CI
- Main benchmark run (all methods, 14 datasets, first settings) running: /root/circuit-learning-results/hclt, notes hclt-NOTES.md, summary summarize_hclt.py
- HCLT failures found and fixed in separate bug-fix PRs: GAS UnmeasurableStepMemoryError -> cram2#1073 (probe doubles rows until memory grows); AD out of memory -> cram2#1075 (rows_per_step measures with the setting's edges, not dense; developer confirmed)
- HCLT rerun queued from /root/hclt-rerun (branch + both fixes, 912ee0a270) by /root/hclt-rerun-after-main.sh into hclt-rerun/

Next
- When the main run ends, watch the HCLT rerun; then summarize hclt + hclt-rerun and add the tables to the PR description (note it depends on #1073 and #1075)
- Plan site dispatch setup skipped by the user (plan_site.publication not run)

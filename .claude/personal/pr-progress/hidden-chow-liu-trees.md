Plan
- Developer decides: rows_per_step measuring an EM step instead of a gradient descent step; mini-batch phase defaults (paper states no step size / batch size); close #1073/#1075 now that both are in #1069?

Done
- Library, benchmark method, review fixes, selectable mutual information estimator, merged cram2 main after #1070
- Big benchmark stopped on request (no performance tables)
- Debugging: GAS UnmeasurableStepMemoryError fixed (0295a4c66a, also cram2#1073); AD out of memory fixed (b0510606c3, also cram2#1075); AD -inf was a double-encoding mistake in my probe, not a bug; HCLT trains on GAS (test LL 0.195) and AD (test LL -63.3) with 2 epochs per phase
- Checked both papers: 12 hidden states and hybrid EM match; paper uses 100+100 fixed epochs and a mixture of 4 HCLTs; no step size / batch size stated; removed the false Einsum credit for 0.1/512 (8c72d736d3, force-pushed reword approved)
- PR description rewritten; labels new-feature, maintenance, bug-fix; neem-4 circuit_learning_test 21 passed at 8c72d736d3

Next
- Wait for the developer's answers above
- Plan site dispatch setup skipped by the user (plan_site.publication not run)

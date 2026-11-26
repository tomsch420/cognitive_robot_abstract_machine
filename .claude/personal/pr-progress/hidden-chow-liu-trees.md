Plan
- Port mutual information (learning/structure/mutual_information.py) from prototype branch circuit-learning-methods, tests first
- Port ChowLiuTree + HiddenChowLiuTree (random parameters only) into learning/structure/hidden_chow_liu_tree/, tests first
- Add a hidden Chow-Liu tree benchmark method to experiments.circuit_learning on main's jax/torch EM (mini-batch then full-batch)
- Run tests in neem-4 container tom-probabilistic-experiments(-ros); krrood format/check; push; update the PR description with deviations from the paper

Done
- Branch hidden-chow-liu-trees off cram/main (has #1050); draft PR cram2#1069 (label new-feature); plan item recorded in progress with repository cram2

Next
- Write the mutual information and Chow-Liu tree tests
- Plan site dispatch setup skipped by the user (plan_site.publication not run)

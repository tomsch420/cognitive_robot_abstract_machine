## Plan
- Pilot done; stop for the user's review before any insolvency rule moves (interest-from-filing-rule next).
## Done
- Plan restructured (landlord pilot replaces the interest pilot), issue #130 commented; draft PR #140.
- fe80b21: legal_basis.py, legal_rule.py; DepositCeilingRule (living space only); three termination ground rules in declared order, commercial threshold as firm convention; process doc. 950 tests passed.
- c4c65bd: renamed DueDateArrears.all_of/at to of_each_due_date/at_end_of_due_date (review threads, replied).
- 99223ee: arrears conditions in EQL (reviewer's rule: predicate only where not EQL or for speed); LetsLivingSpace and deposit ceiling stay predicates; replied on all four threads; rule added to the plan's standing rules. 950 passed.
## Next
- krrood exists bug filed as cram2/cognitive_robot_abstract_machine#903; LetsLivingSpace can become EQL once fixed.
- User's review of #140. Open: legal_basis rendered by repr in verbalization (german-clause-verbalization); mixed leases not weighed in LetsLivingSpace; plan site build fails (reusable workflow not found in basstler).

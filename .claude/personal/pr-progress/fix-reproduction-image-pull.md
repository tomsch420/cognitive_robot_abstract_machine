## PR cram2#997 (fix-reproduction-image-pull): Let the bug reproduction job pull the CI image
- Plan: give the reproduce job of reproduce_bug_reports.yml `packages: read`, so the runner can pull ghcr.io/cram2/cognitive_robot_abstract_machine:jazzy (403 Forbidden in run 44100561697 for cram2#996)
- Done: draft cram2#997 at 01a18da3d3 with bug-fix; actionlint passes
- Next: after merging, edit cram2#996 to rerun the workflow; it should comment with the failing assertion and add `reproduced`

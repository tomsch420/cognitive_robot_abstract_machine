## PR cram2#997 (fix-reproduction-image-pull): Let the bug reproduction job pull the CI image, and keep opencv-python below 5.1
- Plan: give the reproduce job `packages: read` so the runner can pull the CI image (403 for cram2#996); robokudo requires opencv-python<5.1, since 5.1.0.95 has only Windows wheels and breaks every CI install
- Done: both commits on cram2#997 at ad47918677 (pin moved in from the closed cram2#998); a fresh resolution picks opencv-python 5.0.0.93, which has manylinux wheels
- Next: CI of cram2#997; robokudo code owners review the pin; after merging, edit cram2#996 to rerun the workflow; lift the bound once 5.1 has Linux wheels

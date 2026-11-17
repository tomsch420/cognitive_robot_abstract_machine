## PR cram2#998 (pin-opencv-python): Keep opencv-python below 5.1 until it publishes Linux wheels
- Plan: robokudo requires opencv-python<5.1, since 5.1.0.95 (2026-11-17) has only Windows wheels and breaks every CI install
- Done: draft cram2#998 at f9f0417f4e with bug-fix; a fresh resolution picks 5.0.0.93, which has manylinux wheels
- Next: robokudo code owners review; after merging, rerun the failed jobs of cram2#997; lift the bound once 5.1 has Linux wheels

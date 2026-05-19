---
name: parity-workflow
description: How to verify algorithm output didn't drift during refactoring
metadata:
  type: project
---

The strongest regression gate is a byte-level `cmp` of the produced
PCDs against frozen golden PCDs. This is what
`.github/workflows/parity.yml` automates, and what
`tests/scripts/run_parity.sh` does standalone.

**Quick command** (after `prepare_fixtures.sh` has populated everything):

```bash
GITHUB_WORKSPACE=$PWD tests/scripts/run_parity.sh
```

End-to-end ~5 min: builds erasor2 inside docker (catkin cache hits if
unchanged), runs `mapgen` + `run_erasor2`, `cmp`s the outputs against
`~/erasor2_golden/`, then runs `assert_metrics.py` for the PR/RR/F1
tolerance backup.

**When the cmp fails**, that's almost always real. The script saves the
mismatched outputs to `/tmp/erasor2_ci_out/` for inspection. Likely
causes:

1. Algorithm code drift — investigate the diff
2. Intentional algorithm change — refresh the golden:
   `rm -rf ~/erasor2_golden && tests/scripts/prepare_fixtures.sh`
3. PCL or compiler upgrade — bits move, but PR/RR/F1 numbers should
   stay strong; refresh golden and bump tolerance docs if needed

**Why this is meaningful at the byte level**: we proved during v1.0
that pure I/O changes (publisher → rerun, rosparam → yaml-cpp) preserve
mapgen + run_erasor2 output to the last byte, given identical inputs.
So future code changes that ALSO preserve output byte-identically can
be approved by `cmp` alone — no semantic test needed. Reformats,
include reordering, comment changes etc. are obvious wins for `cmp`
gating.

**The PR trigger is disabled at v1**; only `workflow_dispatch` is
enabled, until a self-hosted runner labeled `erasor2-rig` is registered
on the dev machine. See `tests/README.md` for the registration steps.

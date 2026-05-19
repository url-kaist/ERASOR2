---
name: project-profile
description: Who works on ERASOR2 and what the repo is for
metadata:
  type: project
---

User is Hyungtae Lim (KAIST / MIT-affiliated). Email `shapelim@mit.edu`.
Original author of ERASOR (v1) and ERASOR2; the github org is
`LimHyungTae`.

ERASOR2 removes dynamic objects from accumulated LiDAR maps for
SemanticKITTI / HeLiPR datasets. The algorithm is grid-map-based,
log-odds region proposal + volumetric outlier removal. Reference
performance on SemanticKITTI seq-05 [2350..2670] with interval=2,
voxel=0.2: PR=97.668 %, RR=98.457 %, F1=0.9806.

**Long-term goal not yet executed**: bring the original ERASOR (v1)
algorithm into this repo as a sibling (`erasor1::OfflineMapUpdater`
+ `erasor1::ERASOR`) sharing the dataloader/Config/RerunLogger
infrastructure, eventually retiring the standalone ERASOR repo.
This is the natural v1.1 / v1.2 target. See [[migration-history]] for
what already shipped in v1.0.

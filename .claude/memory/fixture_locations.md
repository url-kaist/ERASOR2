---
name: fixture-locations
description: Where the test data and golden PCDs live on disk (NOT in git)
metadata:
  type: reference
---

The parity check needs ~880 MB of data that is gitignored. Locations on
the original dev machine (`shapelim@<original>`):

| Path | Size | What |
|---|---|---|
| `~/catkin_ws_for_erasor2/data/kitti_semantic/dataset/sequences/05/velodyne/` | 611 MB | 321 .bin scans, frames 2350..2670 |
| `…/05/labels/` | 154 MB | SemanticKITTI ground-truth labels for the subset |
| `…/05/{calib,times,suma_pose}.txt` | <1 MB | Calibration + pose files |
| `…/05/hdbscan/` | 154 MB | per-frame instance labels (produced by `kitti_clustering.py`) |
| `…/05/patchwork/` | 154 MB | per-frame ground labels (same) |
| `~/erasor2_golden/05_2350_to_2670_w_interval_2_voxel_0_2.pcd` | 29 MB | reference GT map |
| `~/erasor2_golden/05_0_frame_2350_to_2670_estimated.pcd` | 27 MB | reference estimated map |

The original SemanticKITTI tree on the dev machine lives at
`/media/shapelim/UX960NVMe11/kitti_semantic/dataset/sequences/05/` (full
2761 frames + multiple pose variants). The `data/` directory is a
trimmed copy for fast access.

On a fresh machine, two ways to populate:

1. **Generate locally** via `tests/scripts/prepare_fixtures.sh` (needs
   the full SemanticKITTI seq-05 + conda env `erasor2-3.10`)
2. **Transfer the bundle** via the `erasor2_transfer.tar.gz` tarball
   produced for cross-machine setup (~489 MB compressed, 1.1 GB
   extracted). See [[migration-history]] for context.

The docker container on the original machine is named `erasor2_dev`
and mounts the workspace at `/home/catkin_ws`.

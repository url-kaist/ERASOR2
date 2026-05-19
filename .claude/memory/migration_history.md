---
name: migration-history
description: What v1.0 did (ROS-free runtime) and what it preserved
metadata:
  type: project
---

**v1.0 (2026-05-19, tag `v1.0.0`)** — first ROS-free release. The runtime
no longer needs a ROS master.

What changed:

- yaml-cpp replaces rosparam for all configuration loading
- rerun.io C++ SDK 0.21 replaces every RViz / tf2 / publisher call
- All ROS message types (`sensor_msgs`, `geometry_msgs`, `nav_msgs`,
  `visualization_msgs`, `grid_map_msgs`, `tf2`, `cv_bridge`,
  `pcl_conversions`, etc.) removed from build deps
- New static lib `erasor2_deros` (Config + RerunLogger) owns yaml parsing
  and rerun logging
- Publisher shim classes (`erasor2::viz::Publisher`,
  `GridMapPublisher`, `TextArrayPublisher`) keep `.publish(...)` call
  sites unchanged

What stayed:

- Algorithm code — log-odds, region proposal, VoR adaptive voxel sizing,
  ground likelihood, instance scoring — **byte-identical** to the
  pre-migration source. Verified by running rerun-branch and legacy ROS1
  builds against the same SemanticKITTI seq-05 [2350..2670] input;
  `cmp` returned 0 on both the mapgen PCD and the estimated PCD, and on
  all 161 per-frame MOS labels.
- `grid_map_core` / `grid_map_cv` — kept as the only ROS-derived deps
  (pure C++/Eigen libs that happen to ship as catkin packages)

Parity-check CI lives at `.github/workflows/parity.yml` but trigger is
disabled until a self-hosted runner is registered. See
[[parity-workflow]].

Stash recovery lesson: `git stash push -u` failed silently mid-cleanup
on root-owned files (the docker had written output PCDs as root). The
"dropped" stash was still recoverable via `git fsck --unreachable` →
`git show <stash-commit>:<path>` to extract files. For tracked-only
work, use `git stash push -- <pathspec>` instead.

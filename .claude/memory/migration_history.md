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

**v1.1 (2026-05-20)** — full catkin removal. Codebase is pure C++ now.

**Why:** Hyungtae wanted the repo to build on any distro, not just
Ubuntu 20.04 with ROS Noetic. The `ros-noetic-grid-map-*` packages were
the last toolchain anchor pinning the project to noetic.

**How to apply:** when modifying the build, expect a vanilla
`cmake -B build && cmake --build build` flow. No `catkin build`, no
`source devel/setup.bash`. The docker image is still
`shapelim/opengl-ubuntu20.04-erasor2:latest` but it's used as a host for
the deps (PCL 1.10, Eigen, OpenCV) — the catkin wrapper is unused.

What v1.1 did:

- Wrote `include/erasor2/grid_map.hpp` + `src/erasor2/grid_map.cpp`
  (~200 LOC) replicating the slice of `grid_map_core` / `grid_map_cv`
  the algorithm actually uses (ctor, setGeometry, setPosition, at,
  operator[], get, getIndex, exists, isInside; plus
  `GridMapCvConverter::{toImage,addLayerFromImage}<unsigned char, 1>`).
  Index math (`getIndexFromPosition` with buffer start (0, 0)) mirrors
  upstream 1.6.4 exactly; we never call `move()`, so the circular
  buffer is inert.
- Renamed `grid_map::` → `erasor2::` at every call site.
- Deleted `package.xml`, the 12 dead-ROS files in `src/erasor2/` and
  `src/example/`, the stale `pcl_conversions`/`pcl_ros` includes in
  the dataprocessor.
- Rewrote `CMakeLists.txt` without `find_package(catkin)` or
  `catkin_package()`. New build dep list: PCL ≥ 1.7, Eigen3, OpenCV,
  OpenMP, Boost, yaml-cpp. Plus `rerun_sdk` via FetchContent (already
  there since v1.0).
- Bit-exact parity confirmed by `cmp` against the v1.0 golden PCDs
  (mapgen + run_erasor2 outputs on KITTI seq-05 [2350..2670]).

Parity-check CI lives at `.github/workflows/parity.yml` but trigger is
disabled until a self-hosted runner is registered. The parity script
was updated to use plain cmake instead of catkin. See
[[parity-workflow]].

Stash recovery lesson: `git stash push -u` failed silently mid-cleanup
on root-owned files (the docker had written output PCDs as root). The
"dropped" stash was still recoverable via `git fsck --unreachable` →
`git show <stash-commit>:<path>` to extract files. For tracked-only
work, use `git stash push -- <pathspec>` instead.

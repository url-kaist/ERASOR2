# ERASOR2

ERASOR2 removes dynamic objects from accumulated LiDAR maps. **As of v1.0
the runtime no longer requires a ROS master.** Configuration is loaded
from YAML; visualization is logged to [rerun.io](https://rerun.io). The
only ROS-derived dependencies that remain are `grid_map_core` and
`grid_map_cv` — pure C++/Eigen libraries that happen to ship as catkin
packages.

If you're coming from the ROS1 era and looking for `roslaunch` + RViz —
see the [migration note](#migration-from-the-ros1-era) below.

______________________________________________________________________

## Quick start

```bash
# 1. Build (inside the project's docker image, or any Ubuntu 20.04
#    host with PCL 1.10 + libyaml-cpp-dev + ros-noetic-grid-map-{core,cv}).
cd /home/catkin_ws        # workspace must contain src/ERASOR2/
catkin build erasor2
source devel/setup.bash

# 2. Generate dynamic / ground per-frame labels for your sequence
#    (uses scripts/kitti_clustering.py inside a conda env with open3d +
#    pypatchworkpp; see scripts/environment.yml).
python scripts/kitti_clustering.py --seq 05 --init_stamp 2350 \
       --end_stamp 2670 --save-instance-labels --save-ground-labels

# 3. Build the ground-truth map and run ERASOR2 in one shot:
python scripts/run_pipeline.py --config config/seq_05.yaml \
       --conda-env ~/.miniconda3/envs/erasor2-3.10
```

The Python wrapper just orchestrates the C++ binaries via `subprocess`
plus the existing `evaluate.py`; you can also invoke the binaries
directly:

```bash
mapgen      config/seq_05.yaml
run_erasor2 config/seq_05.yaml
python scripts/evaluate.py --gt <abs_save_dir>/<gt>.pcd \
                           --est <abs_save_dir>/<est>.pcd
```

## Prerequisites

| Layer | Component | Notes |
|---|---|---|
| Build | PCL ≥ 1.7, OpenCV, OpenMP, Eigen3, Boost | All standard Ubuntu packages |
| Build | `libyaml-cpp-dev` | Replaces rosparam |
| Build | `ros-$ROS_DISTRO-grid-map-core`, `…-grid-map-cv` | C++ only, no roscore at runtime |
| Build | `rerun_sdk` 0.21 | **Fetched by CMake** at configure time — no system install |
| Runtime (optional) | [rerun viewer](https://rerun.io/docs/getting-started/installing-viewer) | Only if you want a live GUI; otherwise set `rerun.spawn: false` and inspect the saved `.rrd` later |
| Preprocessing / eval | conda env from `scripts/environment.yml` | open3d + pypatchworkpp + hdbscan + sklearn |

A pre-built docker image is published as
`shapelim/opengl-ubuntu20.04-erasor2:latest` if you'd rather not install
the system layer yourself.

## Configuration

Each binary takes one positional argument: the path to a YAML. Any key
omitted from the YAML falls back to the default in
`include/erasor2/Config.hpp`. Schema highlights:

```yaml
start_frame: 2350
end_frame:   2670
is_large_scale: true

dataloader:
  dataset_name: "SemanticKITTI"          # or "HeLiPR"
  abs_data_dir: "/path/to/sequences"
  sequence:    "05"
  abs_save_dir: "/path/to/output"
  instance_seg_method: "hdbscan"         # or "cais"
  accum_interval: 2
  voxel_size:     0.2
  map_voxel_size: 0.2

erasor2:
  grid_resolution:     2.0
  range_of_interest:   60.0
  min_z_voi:          -3.0
  max_z_voi:           1.5
  scan_ratio_threshold: 0.2
  log_odds:            { increment_gain: 2.0, increment: 0.15 }
  region_proposal_thr: 0.8
  moving_object_detection:
    obj_score_soft_thr: 0.8
    obj_score_hard_thr: 14.0
    hard_thr_radius:    10.0
  viz_flag:                              # all default to false
    set_scan_and_pose: false
    update: false
    detect: false
    over_seg: false
  save_map: true

extrinsic:
  robot_body_size: 2.7
  sensor_height:   1.73
  rotation:    [1, 0, 0,  0, 1, 0,  0, 0, 1]
  translation: [0, 0, 0]

# NEW since v1.0 — visualization sink.
rerun:
  enabled:   true   # set false to make every log call a no-op
  spawn:     true   # launch the rerun viewer subprocess on init
  save_path: ""     # if non-empty, dump a .rrd recording instead of
                    # spawning a viewer (useful for CI / headless runs)
```

Per-sequence configs live in `config/` (`seq_05.yaml`, `seq_07.yaml`, …).

## Binaries

| Binary | Purpose |
|---|---|
| `mapgen` | Accumulate raw scans into a labelled GT map |
| `run_erasor2` | Remove dynamic objects, produce the static map + per-frame MOS labels |
| `compare_map` | Compare multiple estimates against ground truth (TP/FP/FN/TN) |
| `accum_4dmos` | Accumulate 4D-MOS predictions into a static map |
| `fill_removert_labels` | Fill REMOVERT label files (positional args, no YAML) |
| `helipr_to_kitti`, `merge_heliclouds` | HeLiPR dataset preprocessing |

All seven take a single YAML config argument, except
`fill_removert_labels` which takes two positional PCD paths.

## Headless / batch operation

If no viewer is desired (CI, batch evaluation, paper-figure generation),
point `rerun.save_path` at a `.rrd` file and set `spawn: false`:

```yaml
rerun:
  enabled:   true
  spawn:     false
  save_path: /tmp/erasor2_run.rrd
```

Inspect later with `rerun /tmp/erasor2_run.rrd`. Setting
`rerun.enabled: false` makes every visualization call a no-op (slightly
faster end-to-end runtime).

## Reproducing the paper

Reference: SemanticKITTI sequence 05, frames 2350–2670, interval 2,
voxel 0.2. Expected numbers on this subset:

| Metric | Value |
|---|---|
| Preservation Rate (static recall) | 97.668 % |
| Removal Rate (dynamic removal) | 98.457 % |
| F1 | 0.9806 |

To reproduce: run `scripts/kitti_clustering.py` to generate the cais /
hdbscan / patchwork labels, then `python scripts/run_pipeline.py --config config/seq_05.yaml`. The same subset is what the parity-check
CI workflow asserts byte-identical against (see `tests/README.md`).

## Migration from the ROS1 era

`roslaunch erasor2 run_erasor2.launch target_seq:=seq_05` is gone.
Equivalent:

```bash
run_erasor2 ./config/seq_05.yaml
```

The `launch/` directory is kept for historical reference only and is
not wired into the build.

## Repository layout

```
config/             per-sequence YAML configs
include/            public headers
src/                C++ sources for the seven binaries + shared utils
scripts/            Python helpers (preprocessing, evaluation, pipeline driver)
tests/              parity-check CI (workflow disabled until self-hosted runner is registered)
launch/             [legacy] ROS1 launch files, no longer used by the build
rviz/               [legacy] RViz configs, replaced by rerun entity tree
```

## Citation

If you use this code, please cite the ERASOR / ERASOR2 papers. (See the
project page on github.com/LimHyungTae/ERASOR2 for the BibTeX entries.)

## License

GPLv3 — see the `LICENSE` file.

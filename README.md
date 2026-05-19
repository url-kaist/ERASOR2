# ERASOR2

> **`rerun` branch — ROS-free runtime.** The runtime no longer publishes to a
> ROS master. Visualization is via [rerun.io](https://rerun.io); configuration
> is loaded from a YAML file. The only ROS-derived dependency that survives is
> `grid_map_core` / `grid_map_cv` — pure C++ libraries pulled in via catkin
> for the in-memory 2D grid data structure used by the algorithm.

## Prerequisites

### System dependencies

- PCL (Point Cloud Library)
- OpenCV
- OpenMP
- yaml-cpp (`libyaml-cpp-dev` on Ubuntu)
- `grid_map_core`, `grid_map_cv` — currently shipped via catkin packages
  (`ros-$ROS_DISTRO-grid-map-core`, `ros-$ROS_DISTRO-grid-map-cv`). No
  `roscore` is needed at runtime.
- The [rerun viewer](https://rerun.io/docs/getting-started/installing-viewer)
  if you want a live GUI; otherwise set `rerun.spawn: false` in your config
  and load the saved `.rrd` file later.

`rerun_sdk` (the C++ logger SDK) is fetched at configure time by CMake from
the official GitHub release — no system install required.

### Build

```bash
cd /path/to/your/catkin_ws        # workspace must contain ERASOR2/
catkin build erasor2              # or: catkin_make
source devel/setup.bash
```

## Configuration

Configs are plain YAML in `config/`. Key sections:

```yaml
erasor2:
  grid_resolution: 2.0
  range_of_interest: 60.0
  min_z_voi: -4.0
  max_z_voi: 1.5
  scan_ratio_threshold: 0.2
  log_odds: { increment_gain: 1.0, increment: 0.3 }
  region_proposal_thr: 0.8
  moving_object_detection: { obj_score_soft_thr: 0.8, obj_score_hard_thr: 20.0,
                             hard_thr_radius: 20.0 }
  volumetric_outlier_removal: { window_size: 1, dist_thr_gain: 1.732 }
  viz_flag: { set_scan_and_pose: false, set_submap: false, update: false,
              detect: true, over_seg: false }

dataloader:
  dataset_name: SemanticKITTI         # or HeLiPR
  abs_data_dir: /path/to/dataset
  sequence: "05"
  abs_save_dir: /path/to/output
  start_frame: 2350
  end_frame: 2670
  accum_interval: 2
  voxel_size: 0.05
  map_voxel_size: 0.2

extrinsic:
  robot_body_size: 2.7
  sensor_height: 1.73
  rotation: [1, 0, 0,  0, 1, 0,  0, 0, 1]
  translation: [0, 0, 0]

# NEW — visualization is now driven by rerun, not RViz.
rerun:
  enabled: true        # set false to disable all viz
  spawn: true          # launch the rerun viewer subprocess on init
  save_path: ""        # if non-empty, write a .rrd recording instead of spawning
```

Any key not present in the YAML falls back to the default in
`include/erasor2/Config.hpp`.

## Running

All binaries take exactly one positional argument: the path to a config YAML.

```bash
# Static map generation
mapgen   config/seq_05.yaml

# ERASOR2 — dynamic-object removal & static map building
run_erasor2 config/seq_05.yaml

# Compare estimates against ground truth (TP/FP/FN/TN per method)
compare_map config/compare_map.yaml

# Accumulate 4D-MOS labels into a static map
accum_4dmos config/seq_05.yaml [target_mos_type]

# Fill REMOVERT labels (positional args, no YAML)
fill_removert_labels <raw.pcd> <removert.pcd>

# HeLiPR pre-processing
helipr_to_kitti  config/HeLiPR.yaml
merge_heliclouds config/HeLiPR.yaml
```

Once the rerun viewer is running, every binary that does visualization will
log into the same recording session. Frame indices appear on the timeline
slider; the entity tree mirrors the legacy RViz topic names
(`erasor2/curr_scan`, `erasor2/static`, `world/path`, `world/body`, …).

## Headless operation

If you don't want the viewer to pop up (CI, batch evaluation), point
`rerun.save_path` at a `.rrd` file:

```yaml
rerun:
  enabled: true
  spawn: false
  save_path: /tmp/erasor2_run.rrd
```

Open the `.rrd` later with `rerun /tmp/erasor2_run.rrd`.

## Migration note (legacy users)

`roslaunch erasor2 run_erasor2.launch target_seq:=seq_05` is gone. The
equivalent now is:

```bash
run_erasor2 $(rospack find erasor2)/config/seq_05.yaml   # ROS-aware shell
run_erasor2 ./config/seq_05.yaml                          # plain shell
```

Old `.launch` files in `launch/` are kept for reference but no longer wired
into the build. They will be removed in a follow-up cleanup.

## Key parameters

- `grid_resolution`: Grid map resolution
- `range_of_interest`: Maximum detection range
- `min_z_voi`, `max_z_voi`: Vertical region of interest (in raw cloud frame)
- `scan_ratio_threshold`: Detection sensitivity (higher = more aggressive)

## Data format

- Point clouds in PCD or `.bin` (KITTI binary)
- Trajectory/poses in the loader's expected format
- Per-frame instance / ground labels for SemanticKITTI runs (see
  `scripts/kitti_clustering.py`)

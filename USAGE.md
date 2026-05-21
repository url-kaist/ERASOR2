# ERASOR2 — Usage

Operational guide for ERASOR2: installation, running the pipeline on
SemanticKITTI / HeLiPR / HeLiMOS, visualization, YAML schema, and the
common gotchas accumulated while porting the project from
ROS+catkin to plain CMake.

The headline numbers and the minimal seq-05 reproduction quickstart
live in the top-level [`README.md`](README.md); everything below is
the deeper reference.

______________________________________________________________________

## :package: Installation

> ERASOR2 builds from a single `cmake` invocation on any Linux distro
> (or macOS) with PCL, Eigen, OpenCV, OpenMP, Boost, and yaml-cpp.
> No catkin workspace, no ROS environment.

### System packages

```bash
# Ubuntu 20.04 / 22.04
sudo apt-get install -y \
    build-essential cmake \
    libpcl-dev libeigen3-dev libopencv-dev libomp-dev \
    libboost-system-dev libboost-filesystem-dev \
    libyaml-cpp-dev
```

The `rerun_sdk` 0.32 dependency is **fetched by CMake at configure
time** — no system install needed.

### C++

```bash
git clone https://github.com/LimHyungTae/ERASOR2.git
cd ERASOR2
cmake -B build -S .
cmake --build build -j
```

Verified on Ubuntu 20.04 (GCC 9.4 + PCL 1.10) and Ubuntu 22.04
(GCC 11.4 + PCL 1.12) from the same source tree. Seven binaries are
produced under `build/`:

| Binary | Purpose |
|---|---|
| `mapgen` | Accumulate raw scans into a labelled ground-truth map |
| `run_erasor2` | Remove dynamic objects → static map + per-frame MOS labels |
| `compare_map` | Compare multiple estimates against GT (TP/FP/FN/TN) |
| `accum_4dmos` | Accumulate 4D-MOS predictions into a static map |
| `fill_removert_labels` | Fill REMOVERT label files (two positional PCD paths) |
| `helipr_to_kitti`, `merge_heliclouds` | HeLiPR / HeLiMOS preprocessing |

<details>
  <summary><strong>Q. I don't want to install the system layer myself.</strong></summary>

A pre-built image is published at
`shapelim/opengl-ubuntu20.04-erasor2:latest`. Mount the repo and build
inside it:

```bash
docker run --rm -v "$PWD":/work -w /work \
    shapelim/opengl-ubuntu20.04-erasor2:latest \
    bash -c "cmake -B build -S . && cmake --build build -j"
```

</details>

<details>
  <summary><strong>Q. Build fails on Ubuntu 24.04 with <code>target "VTK::mpi" was not found</code>.</strong></summary>

Stale `VTK::mpi` import in the 24.04 PCL config. Add
`find_package(MPI QUIET)` before `find_package(PCL)` in
`CMakeLists.txt`, or build inside the 20.04 docker.

</details>

<details>
  <summary><strong>Q. Where are the ROS launch files / RViz configs?</strong></summary>

`roslaunch erasor2 run_erasor2.launch target_seq:=seq_05` is gone since
v1.0. Equivalent invocation:

```bash
./build/run_erasor2 ./config/erasor2/seq_05.yaml
```

The `launch/` and `rviz/` directories are kept for historical
reference and are not wired into the build. The 2D grid abstraction
that used to come from `ros-noetic-grid-map-*` lives in
`include/erasor2/grid_map.hpp` as of v1.1 — a ~150 LOC subset of the
upstream `grid_map_core` API mirrored to match it bit-for-bit on the
seq-05 parity check. Visualization runs through
[rerun.io](https://rerun.io) instead of RViz / tf2.

</details>

### Python (preprocessing + evaluation)

The C++ binaries consume per-frame instance + ground labels and
PCD outputs that are produced by a handful of Python helpers
(open3d, pypatchworkpp, HDBSCAN). The conda recipe is shipped in
the repo:

```bash
conda env create -f scripts/environment.yml   # creates env "erasor2"
conda activate erasor2
```

Once the env is active, `scripts/run_pipeline.py` autodetects it and
chains preprocessing → mapgen → run_erasor2 → evaluate in one shot
(see [How to run](#rocket-how-to-run)).

<details>
  <summary><strong>Q. open3d import crashes with <code>CXXABI_1.3.15 not found</code>.</strong></summary>

Ubuntu 20.04's system `libstdc++` only goes up to `CXXABI_1.3.14`;
open3d ≥ 0.18 needs `1.3.15`, which lives in the conda env's own
`libstdc++`. Preload it:

```bash
export LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
```

`scripts/run_pipeline.py --conda-env <path>` sets this automatically.

</details>

<details>
  <summary><strong>Q. Why not pure pip?</strong></summary>

`pypatchworkpp` and `open3d` both ship native wheels that need a
recent glibc / libstdc++; mixing them with a system Python on
Ubuntu 20.04 routinely produces ABI mismatches. Conda isolates the
toolchain end-to-end, which is what `run_pipeline.py` expects.

</details>

______________________________________________________________________

## :rocket: How to run

ERASOR2 needs three inputs per sequence: raw scans, per-frame
**instance labels** (HDBSCAN), and per-frame **ground labels**
(Patchwork). A Python helper generates the labels; a Python driver
chains the C++ binaries end-to-end.

```bash
# 1. Per-frame instance + ground labels (conda env w/ open3d + pypatchworkpp).
#    --kitti_dir is the directory ABOVE 'dataset/', so that
#    <kitti_dir>/dataset/sequences/05/velodyne/ exists.
python scripts/kitti_clustering.py \
    --kitti_dir /home/url/datasets/kitti \
    --seq 05 --init_stamp 2350 --end_stamp 2670 \
    --save-instance-labels --save-ground-labels

# 2. Full pipeline: mapgen → run_erasor2 → evaluate.
#    Before running, edit `config/erasor2/seq_05.yaml` and set `abs_data_dir`
#    (= <kitti_dir>/dataset/sequences) and `abs_save_dir` (where mapgen
#    and run_erasor2 will write the GT/estimated PCDs) for your machine.
python scripts/run_pipeline.py \
    --config config/erasor2/seq_05.yaml \
    --conda-env ~/.miniconda3/envs/erasor2-3.10
```

> [!IMPORTANT]
> Each `config/*.yaml` ships with example paths from the maintainer's
> machine. Before running anything that takes a `--config`, open the
> file and update `dataloader.abs_data_dir` + `dataloader.abs_save_dir`
> (and, for HeLiPR, the per-sensor paths) to point at your own dataset
> and output locations. `run_pipeline.py` derives `--kitti_dir` for the
> preprocessing step from `abs_data_dir` automatically (two parents up).

> [!WARNING]
> **Always eyeball the HDBSCAN instance segmentation before running the
> full pipeline.** The clustering quality varies sequence-to-sequence
> and frame-to-frame; bad clusters (over-segmented dynamic objects,
> merged car-plus-ground blobs, missed pedestrians) will silently
> degrade ERASOR2's PR/RR/F1. Use the visualizer below to scrub the
> labels frame-by-frame and confirm distinct objects get distinct
> colors before trusting the downstream numbers.

### Visualizing the clustering output

`scripts/visualize_clustering.py` loads the per-frame velodyne + the
patchwork ground labels + the hdbscan instance labels and streams them
into a [rerun](https://rerun.io) viewer on a frame timeline:

- `world/ground` — Patchwork++ ground points (brown)
- `world/instances` — non-ground points colored by HDBSCAN cluster ID

```bash
# Spawns the rerun viewer. Use the same args you passed to kitti_clustering.py.
python scripts/visualize_clustering.py \
    --kitti_dir /home/url/datasets/kitti \
    --seq 05 --init_stamp 2350 --end_stamp 2670

# Or save a .rrd you can open later with `rerun /path/to/file.rrd`:
python scripts/visualize_clustering.py \
    --kitti_dir /home/url/datasets/kitti \
    --seq 05 --init_stamp 2350 --end_stamp 2670 \
    --save cluster_viz.rrd
```

Drag the timeline slider at the bottom of the viewer to step through
frames. `--show-raw` adds the uncolored raw scan as a third overlay.

The wrapper just orchestrates the C++ binaries via `subprocess`; you
can also invoke them directly:

```bash
./build/mapgen      config/erasor2/seq_05.yaml
./build/run_erasor2 config/erasor2/seq_05.yaml
python scripts/evaluate.py \
    --gt  <abs_save_dir>/<gt>.pcd \
    --est <abs_save_dir>/<est>.pcd
```

<details>
  <summary><strong>YAML schema (highlights)</strong></summary>

Each binary takes one positional argument: the path to a YAML. Any
key omitted falls back to the default in `include/erasor2/Config.hpp`.

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
  save_map: true

extrinsic:
  robot_body_size: 2.7
  sensor_height:   1.73
  rotation:    [1, 0, 0,  0, 1, 0,  0, 0, 1]
  translation: [0, 0, 0]

rerun:
  enabled:   true   # false ⇒ every log call is a no-op
  spawn:     true   # launch the rerun viewer subprocess
  save_path: ""     # if non-empty, dump a .rrd instead of spawning
```

Per-sequence configs live in `config/` (`seq_05.yaml`, `seq_07.yaml`, …).

</details>

<details>
  <summary><strong>Headless / batch operation</strong></summary>

For CI, batch evaluation, or paper-figure generation, point
`rerun.save_path` at a `.rrd` file and set `spawn: false`:

```yaml
rerun:
  enabled:   true
  spawn:     false
  save_path: /tmp/erasor2_run.rrd
```

Inspect later with `rerun /tmp/erasor2_run.rrd`. Setting
`rerun.enabled: false` makes every visualization call a no-op
(slightly faster end-to-end).

</details>

______________________________________________________________________

## :truck: HeLiPR / HeLiMOS

> [!WARNING]
> Not have been tested yet.

The same binaries handle HeLiPR-style data — set
`dataloader.dataset_name` to `"HeLiPR"` and point `abs_data_dir` at
the directory that *contains* the per-sensor trees:

```
<abs_data_dir>/
└── <sensor>/                  # Avia, Aeva, VLP16, or Merged
    ├── velodyne/              # .bin scans
    ├── poses.txt              # KITTI-style 3x4 row-major
    ├── patchwork/             # ground labels (from pypatchworkpp)
    └── hdbscan/               # instance labels (from clusters_hdbscan)
```

`Avia`, `Aeva`, and `VLP16` poses are corrected by the per-sensor
extrinsic baked into `src/dataloader/dataloader.cpp` (`T_OS2_*`).
`Merged` is assumed to be in the Ouster frame and uses identity.

```bash
# Per-frame labels (same script as KITTI; just change --seq).
# For HeLiMOS, --kitti_dir is the directory directly containing the
# sequence folders (e.g. .../HeLiMOS/KAIST05/deskewed_LiDAR/).
python scripts/kitti_clustering.py \
    --kitti_dir /home/url/datasets/HeLiMOS/KAIST05/deskewed_LiDAR \
    --seq Merged --init_stamp 8600 --end_stamp 8649 \
    --save-instance-labels --save-ground-labels

# mapgen + run_erasor2 against the HeLiPR config.
./build/mapgen      config/erasor2/helipr_mapgen.yaml
./build/run_erasor2 config/erasor2/helipr_mapgen.yaml
```

> [!WARNING]
> Same as the KITTI flow: visually inspect a sample of the HDBSCAN
> instance labels before trusting the downstream metrics. HeLiPR sensors
> have very different point densities (Avia/Aeva are far sparser than
> Ouster/Velodyne), so clustering hyperparameters that look fine on one
> sensor may over- or under-segment on another.

The three shipped HeLiPR configs (`config/erasor2/HeLiPR.yaml`,
`config/erasor2/HeLiPR_kitti.yaml`, `config/erasor2/helipr_mapgen.yaml`) carry example
paths from the original dev machines under `/media/...`; repoint
`abs_data_dir` and `abs_save_dir` before running. The HeLiPR-specific
preprocessing binaries `helipr_to_kitti` and `merge_heliclouds`
(per-sensor extraction → time-aligned merge into `Merged`) take their
own YAMLs — see the `dataprocessor:` section in
`config/erasor2/HeLiPR_kitti.yaml`.

______________________________________________________________________

## :warning: Common gotchas

- **`dataloader.expansion_range` defaults to 20.** `run_erasor2` looks
  20 frames *before* each cluster's `start_frame` to expand the
  trajectory submap. For partial copies of a dataset (CI / smoke
  testing), either copy 20 frames of padding before `start_frame`,
  or set `expansion_range: 0` in the YAML.
- **The accumulation loop iterates `[start_frame, end_frame + accum_interval)`.**
  With `end_frame: 2670` and `accum_interval: 2`, the loop reads frame
  2671. Copy at least one extra frame past `end_frame` when using a
  trimmed dataset.
- **`mapgen` prints `[pcl::VoxelGrid] Leaf size too small …`** on
  HeLiMOS-sized maps. Cosmetic — actual voxelization is delegated to
  `voxelize_preserving_labels_by_nanoflann`, which doesn't suffer
  from the integer-index overflow that triggers the PCL warning.

______________________________________________________________________

## :file_folder: Repository layout

```
config/             per-sequence YAML configs
include/            public headers (Config, GridMap, rerun logger, utils)
src/                C++ sources for the seven binaries + shared utils
scripts/            Python helpers (preprocessing, evaluation, pipeline driver)
tests/              parity-check CI (workflow disabled until self-hosted runner is registered)
launch/             [legacy] ROS1 launch files, no longer wired into the build
rviz/               [legacy] RViz configs, replaced by the rerun entity tree
```

______________________________________________________________________

## :handshake: Acknowledgements

ERASOR2 builds on a long line of open-source LiDAR tooling — in
particular [PCL](https://pointclouds.org),
[Eigen](https://eigen.tuxfamily.org), [rerun.io](https://rerun.io),
[pypatchworkpp](https://github.com/url-kaist/patchwork-plusplus), and
[nanoflann](https://github.com/jlblancoc/nanoflann). The v1.0/v1.1
transition that removed ROS and catkin from the runtime would not have
been possible without these dependencies being clean enough to consume
without a ROS environment.

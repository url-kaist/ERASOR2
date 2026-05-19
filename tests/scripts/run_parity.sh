#!/usr/bin/env bash
# Reproduce the seq-05 [2350..2670] parity test inside the project docker
# image. Strict byte-identical cmp against frozen golden PCDs. Optional
# numeric tolerance check on PR/RR/F1 if the conda env is available.
#
# Expects these env vars (the workflow sets them; defaults work for a
# manual local run on the same machine):
#   DOCKER_IMAGE         docker image with ROS noetic + PCL + grid_map_*
#   ERASOR2_DATA_ROOT    .../kitti_semantic/dataset/sequences   (has 05/)
#   ERASOR2_GOLDEN_DIR   dir with the two reference .pcd files
#   ERASOR2_CONDA_ENV    optional; conda env with open3d+sklearn
#
# Source tree must be the working directory (the GitHub Actions checkout).
set -euo pipefail

: "${DOCKER_IMAGE:=shapelim/opengl-ubuntu20.04-erasor2:latest}"
: "${ERASOR2_DATA_ROOT:=/home/shapelim/catkin_ws_for_erasor2/data/kitti_semantic/dataset/sequences}"
: "${ERASOR2_GOLDEN_DIR:=/home/shapelim/erasor2_golden}"
: "${ERASOR2_CONDA_ENV:=/home/shapelim/.miniconda3/envs/erasor2-3.10}"

SEQ="05"
START=2350
END=2670
GT_NAME="${SEQ}_${START}_to_${END}_w_interval_2_voxel_0_2.pcd"
EST_NAME="${SEQ}_0_frame_${START}_to_${END}_estimated.pcd"

SRC_DIR="${GITHUB_WORKSPACE:-$PWD}"
RUN_OUT="/tmp/erasor2_ci_out"
rm -rf "$RUN_OUT" && mkdir -p "$RUN_OUT"

# --- preflight ----------------------------------------------------------
for p in "$ERASOR2_DATA_ROOT/$SEQ/velodyne" \
         "$ERASOR2_DATA_ROOT/$SEQ/labels" \
         "$ERASOR2_DATA_ROOT/$SEQ/hdbscan" \
         "$ERASOR2_DATA_ROOT/$SEQ/patchwork" \
         "$ERASOR2_DATA_ROOT/$SEQ/suma_pose.txt" \
         "$ERASOR2_GOLDEN_DIR/$GT_NAME" \
         "$ERASOR2_GOLDEN_DIR/$EST_NAME"; do
  if [[ ! -e "$p" ]]; then
    echo "Missing fixture: $p" >&2
    echo "Run tests/scripts/prepare_fixtures.sh on the runner host." >&2
    exit 2
  fi
done

# --- CI config ----------------------------------------------------------
cat > "$RUN_OUT/seq_05_ci.yaml" <<EOF
start_frame: $START
end_frame: $END
viz_interval: 10000
is_large_scale: true

dataloader:
    run_traj_clustering: false
    dataset_name: "SemanticKITTI"
    abs_data_dir: "/ci_data/sequences"
    cloud_dir: ""
    cloud_format: ""
    pose_path: ""
    sequence: "$SEQ"
    abs_save_dir: "/ci_out"
    instance_seg_method: "hdbscan"
    accum_interval: 2
    voxel_size: 0.2
    map_voxel_size: 0.2
    expansion_range: 0

erasor2:
    grid_resolution: 2.0
    egocentric_grid_resolution: 0.6
    range_of_interest: 60.0
    min_z_voi: -3.0
    max_z_voi: 1.5
    min_z_diff_thr: 0.4
    scan_ratio_threshold: 0.2
    log_odds:
        increment_gain: 2.0
        increment: 0.15
    region_proposal_thr: 0.8
    kernel_size: 1
    ratio_num_pts: 0.95
    minimum_num_pts: 5
    moving_object_detection:
        negative_log_odds: -2.0
        obj_score_soft_thr: 0.8
        obj_score_hard_thr: 14.0
        hard_thr_radius: 10.0
    over_segmentation:
        minimum_area_thr: 56
        ratio_of_unknown_prior: 0.25
    volumetric_outlier_removal:
        window_size: 1
        use_adaptive_voxel_size: true
        vor_cand_score_thr: 3.0
        dist_thr_gain: 1.732
    viz_flag:
        set_scan_and_pose: false
        set_submap: false
        update: false
        detect: false
        over_seg: false
    save_map: true

stop_for_each_frame: false

extrinsic:
    robot_body_size: 2.7
    sensor_height: 1.73
    rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    translation: [0.0, 0.0, 0.0]

rerun:
    enabled: false
    spawn: false
    save_path: ""
EOF

# --- build + run inside the project docker image ------------------------
# Cache the catkin build dir across runs via a named docker volume so
# incremental PRs don't pay the full 70-second cold build + rerun_sdk
# download. The cache is invalidated automatically by CMake when CMakeLists
# changes; manual `docker volume rm erasor2_ci_build` resets it.
echo "[parity] Running docker build + run_erasor2..."
docker run --rm \
  -v "$SRC_DIR:/ci_src:ro" \
  -v "$ERASOR2_DATA_ROOT:/ci_data/sequences:ro" \
  -v "$RUN_OUT:/ci_out" \
  -v "erasor2_ci_build:/tmp/ci_ws" \
  "$DOCKER_IMAGE" \
  bash -c '
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    set -u
    mkdir -p /tmp/ci_ws/src
    # If the symlink already exists from a cached run, replace it so it
    # points at the new checkout (otherwise catkin reuses stale paths).
    rm -f /tmp/ci_ws/src/ERASOR2
    ln -s /ci_src /tmp/ci_ws/src/ERASOR2
    cd /tmp/ci_ws
    catkin build erasor2 --no-status 2>&1 | tail -5
    set +u
    source devel/setup.bash
    set -u
    rosrun erasor2 mapgen      /ci_out/seq_05_ci.yaml
    rosrun erasor2 run_erasor2 /ci_out/seq_05_ci.yaml
  '

# --- strict byte-identical check ---------------------------------------
echo "[parity] Comparing mapgen PCD..."
if cmp "$RUN_OUT/$GT_NAME" "$ERASOR2_GOLDEN_DIR/$GT_NAME"; then
    echo "  -> BYTE-IDENTICAL ($GT_NAME)"
else
    echo "  -> MISMATCH ($GT_NAME)" >&2
    exit 1
fi

echo "[parity] Comparing estimated PCD..."
if cmp "$RUN_OUT/$EST_NAME" "$ERASOR2_GOLDEN_DIR/$EST_NAME"; then
    echo "  -> BYTE-IDENTICAL ($EST_NAME)"
else
    echo "  -> MISMATCH ($EST_NAME)" >&2
    exit 1
fi

# --- numeric tolerance (soft check) ------------------------------------
# Skipped if the conda env is unavailable. Catches drift that survives a
# legit PCL/compiler upgrade where bits change but metrics shouldn't.
PY_BIN="$ERASOR2_CONDA_ENV/bin/python"
LD_LIB="$ERASOR2_CONDA_ENV/lib/libstdc++.so.6"
if [[ -x "$PY_BIN" && -f "$LD_LIB" ]]; then
    echo "[parity] Running numeric tolerance check..."
    LD_PRELOAD="$LD_LIB" "$PY_BIN" "$SRC_DIR/tests/scripts/assert_metrics.py" \
        --gt  "$RUN_OUT/$GT_NAME" \
        --est "$RUN_OUT/$EST_NAME" \
        --pr-min 97.5 --rr-min 98.0 --f1-min 0.97
else
    echo "[parity] (skipping numeric check: $PY_BIN not found)"
fi

echo "[parity] ALL CHECKS PASSED"

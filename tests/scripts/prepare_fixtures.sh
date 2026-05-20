#!/usr/bin/env bash
# One-shot setup for the self-hosted parity runner.
#
# Run on the runner HOST (not inside docker). Populates:
#   $ERASOR2_DATA_ROOT/05/                  fixture dataset (~820 MB)
#   $ERASOR2_GOLDEN_DIR/                    two golden PCDs (~57 MB)
#
# Steps:
#   1. Copy seq-05 [2350..2670] velodyne + labels + pose files from a
#      SemanticKITTI tree you already have on disk.
#   2. Generate cais/hdbscan instance labels + patchwork ground labels via
#      scripts/kitti_clustering.py (conda env with open3d + pypatchworkpp).
#   3. Build erasor2 inside the project docker and run mapgen + run_erasor2
#      once to produce the golden PCDs. (These represent "what HEAD outputs
#      today"; refresh after intentional algorithm changes.)
#
# Override paths by exporting the variables before invoking.
set -euo pipefail

: "${ERASOR2_SRC_KITTI:=/media/shapelim/UX960NVMe11/kitti_semantic/dataset/sequences/05}"
: "${ERASOR2_DATA_ROOT:=/home/shapelim/catkin_ws_for_erasor2/data/kitti_semantic/dataset/sequences}"
: "${ERASOR2_GOLDEN_DIR:=/home/shapelim/erasor2_golden}"
: "${ERASOR2_CONDA_ENV:=/home/shapelim/.miniconda3/envs/erasor2-3.10}"
: "${DOCKER_IMAGE:=shapelim/opengl-ubuntu20.04-erasor2:latest}"
: "${WORKSPACE_ROOT:=/home/shapelim/catkin_ws_for_erasor2}"

SEQ="05"
START=2350
END=2670
DST="$ERASOR2_DATA_ROOT/$SEQ"

# --- 1. copy velodyne + labels + metadata -------------------------------
if [[ ! -d "$DST/velodyne" || $(ls "$DST/velodyne" 2>/dev/null | wc -l) -lt $((END-START+1)) ]]; then
    echo "[prepare] Copying frames $START..$END from $ERASOR2_SRC_KITTI"
    mkdir -p "$DST/velodyne" "$DST/labels"
    cp "$ERASOR2_SRC_KITTI"/{calib.txt,poses.txt,times.txt} "$DST/" 2>/dev/null || true
    cp "$ERASOR2_SRC_KITTI/poses_suma.txt" "$DST/suma_pose.txt"
    for i in $(seq "$START" "$END"); do
        f=$(printf "%06d" "$i")
        cp "$ERASOR2_SRC_KITTI/velodyne/$f.bin"   "$DST/velodyne/"
        cp "$ERASOR2_SRC_KITTI/labels/$f.label"   "$DST/labels/"
    done
    echo "[prepare] Velodyne: $(ls "$DST/velodyne" | wc -l) bin / Labels: $(ls "$DST/labels" | wc -l) label"
else
    echo "[prepare] Dataset already populated; skipping copy"
fi

# --- 2. generate cais/hdbscan + patchwork labels ------------------------
if [[ ! -d "$DST/hdbscan" || $(ls "$DST/hdbscan" 2>/dev/null | wc -l) -lt $((END-START+1)) ]]; then
    echo "[prepare] Running kitti_clustering.py (open3d + pypatchworkpp + hdbscan)..."
    SCRIPT="$WORKSPACE_ROOT/src/ERASOR2/scripts/kitti_clustering.py"
    cp "$SCRIPT" "$SCRIPT.orig"
    sed -i -E "s|ABS_DATA_DIR = \"/media/shapelim/UX9803/erasor2_test_benchmark/sequences\"|ABS_DATA_DIR = \"$ERASOR2_DATA_ROOT\"|;
               s|ABS_SAVE_DIR = \"/media/shapelim/UX9803/erasor2_test_benchmark/sequences\"|ABS_SAVE_DIR = \"$ERASOR2_DATA_ROOT\"|" "$SCRIPT"
    trap 'mv "$SCRIPT.orig" "$SCRIPT"' EXIT
    cd "$(dirname "$SCRIPT")"
    LD_PRELOAD="$ERASOR2_CONDA_ENV/lib/libstdc++.so.6" \
      "$ERASOR2_CONDA_ENV/bin/python" "$SCRIPT" \
      --seq "$SEQ" --init_stamp "$START" --end_stamp "$END" \
      --save-instance-labels --save-ground-labels
    cd -
    mv "$SCRIPT.orig" "$SCRIPT"
    trap - EXIT
else
    echo "[prepare] Instance/ground labels already populated; skipping"
fi

# --- 3. produce the golden PCDs by running the current HEAD ------------
mkdir -p "$ERASOR2_GOLDEN_DIR"
GT_NAME="${SEQ}_${START}_to_${END}_w_interval_2_voxel_0_2.pcd"
EST_NAME="${SEQ}_0_frame_${START}_to_${END}_estimated.pcd"
if [[ -f "$ERASOR2_GOLDEN_DIR/$GT_NAME" && -f "$ERASOR2_GOLDEN_DIR/$EST_NAME" ]]; then
    echo "[prepare] Golden PCDs already present; skipping regeneration."
    echo "          Delete $ERASOR2_GOLDEN_DIR to force refresh."
    exit 0
fi

echo "[prepare] Generating golden PCDs from current source..."
TMP_OUT=$(mktemp -d)
trap "rm -rf $TMP_OUT" EXIT
cat > "$TMP_OUT/seq_05_golden.yaml" <<EOF
start_frame: $START
end_frame: $END
viz_interval: 10000
is_large_scale: true
dataloader:
    run_traj_clustering: false
    dataset_name: "SemanticKITTI"
    abs_data_dir: "/ci_data/sequences"
    sequence: "$SEQ"
    abs_save_dir: "/ci_out"
    instance_seg_method: "hdbscan"
    accum_interval: 2
    voxel_size: 0.2
    map_voxel_size: 0.2
    expansion_range: 0
    cloud_dir: ""
    cloud_format: ""
    pose_path: ""
erasor2:
    grid_resolution: 2.0
    egocentric_grid_resolution: 0.6
    range_of_interest: 60.0
    min_z_voi: -3.0
    max_z_voi: 1.5
    min_z_diff_thr: 0.4
    scan_ratio_threshold: 0.2
    log_odds: {increment_gain: 2.0, increment: 0.15}
    region_proposal_thr: 0.8
    kernel_size: 1
    ratio_num_pts: 0.95
    minimum_num_pts: 5
    moving_object_detection: {negative_log_odds: -2.0, obj_score_soft_thr: 0.8, obj_score_hard_thr: 14.0, hard_thr_radius: 10.0}
    over_segmentation: {minimum_area_thr: 56, ratio_of_unknown_prior: 0.25}
    volumetric_outlier_removal: {window_size: 1, use_adaptive_voxel_size: true, vor_cand_score_thr: 3.0, dist_thr_gain: 1.732}
    viz_flag: {set_scan_and_pose: false, set_submap: false, update: false, detect: false, over_seg: false}
    save_map: true
stop_for_each_frame: false
extrinsic:
    robot_body_size: 2.7
    sensor_height: 1.73
    rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    translation: [0.0, 0.0, 0.0]
rerun: {enabled: false, spawn: false, save_path: ""}
EOF

docker run --rm \
  -v "$WORKSPACE_ROOT/src/ERASOR2:/ci_src:ro" \
  -v "$ERASOR2_DATA_ROOT:/ci_data/sequences:ro" \
  -v "$TMP_OUT:/ci_out" \
  "$DOCKER_IMAGE" \
  bash -c '
    set -euo pipefail
    cmake -B /tmp/golden_build -S /ci_src
    cmake --build /tmp/golden_build -j"$(nproc)"
    /tmp/golden_build/mapgen      /ci_out/seq_05_golden.yaml
    /tmp/golden_build/run_erasor2 /ci_out/seq_05_golden.yaml
  '

cp "$TMP_OUT/$GT_NAME"  "$ERASOR2_GOLDEN_DIR/"
cp "$TMP_OUT/$EST_NAME" "$ERASOR2_GOLDEN_DIR/"
chmod 644 "$ERASOR2_GOLDEN_DIR"/*.pcd
echo "[prepare] Golden PCDs written to $ERASOR2_GOLDEN_DIR/"
ls -lh "$ERASOR2_GOLDEN_DIR/"

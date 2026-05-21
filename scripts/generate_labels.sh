#!/usr/bin/env bash
# Generate per-frame Patchwork ground labels + HDBSCAN instance labels for
# the five SemanticKITTI sequences in the paper benchmark (00, 01, 02, 05, 07).
# Output lands under <kitti_dir>/dataset/sequences/<seq>/{patchwork,hdbscan}/.
#
# Usage:
#     scripts/generate_labels.sh /path/to/kitti
#
# Requires the `erasor2` conda env to be active (open3d + pypatchworkpp +
# scikit-learn). On Ubuntu 20.04 also set
#     LD_PRELOAD="$CONDA_PREFIX/lib/libstdc++.so.6"
# before invoking; 22.04 doesn't need it.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <kitti_dir>" >&2
  echo "  <kitti_dir>: directory ABOVE 'dataset/' (so that" >&2
  echo "               <kitti_dir>/dataset/sequences/05/velodyne/ exists)" >&2
  exit 1
fi

KITTI_DIR="$1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

declare -A RANGES=(
  ["00"]="4390 4530"
  ["01"]=" 150  250"
  ["02"]=" 860  950"
  ["05"]="2350 2670"
  ["07"]=" 630  820"
)

for seq in 00 01 02 05 07; do
  read -r init_stamp end_stamp <<<"${RANGES[$seq]}"
  echo "=== seq $seq frames $init_stamp..$end_stamp ==="
  python "$SCRIPT_DIR/kitti_clustering.py" \
    --kitti_dir "$KITTI_DIR" \
    --seq "$seq" \
    --init_stamp "$init_stamp" \
    --end_stamp "$end_stamp" \
    --save-instance-labels \
    --save-ground-labels
done

echo "Done. Labels written under $KITTI_DIR/dataset/sequences/<seq>/{patchwork,hdbscan}/"

"""Visualize the output of kitti_clustering.py in rerun.

Reads the raw scans + the patchwork ground labels + the hdbscan instance
labels that kitti_clustering.py wrote, and logs three point clouds per
frame into rerun on a sequence timeline:

    world/raw         all velodyne points, grey
    world/ground      ground points, brown
    world/instances   non-ground points, colored by HDBSCAN instance ID

Use the same --kitti_dir / --save_dir / --seq / --init_stamp / --end_stamp
arguments you passed to kitti_clustering.py.

Example (SemanticKITTI seq-05, labels saved next to velodyne/):

    python scripts/visualize_clustering.py \\
        --kitti_dir /home/url/datasets/kitti \\
        --seq 05 --init_stamp 2350 --end_stamp 2670
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import rerun as rr
from tqdm import tqdm

GROUND_LABEL = 1  # matches V2 in kitti_clustering.py


def resolve_dirs(kitti_dir, save_dir, seq):
    kitti_dir = kitti_dir.rstrip("/")
    if seq == "Merged":
        data_base = kitti_dir + "/" + seq
    else:
        data_base = kitti_dir + "/dataset/sequences/" + seq
    save_base = save_dir.rstrip("/") if save_dir else data_base
    return data_base, save_base


def load_scan(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]


def decode_instance_labels(path):
    # kitti_clustering.py writes uint32 with `(pred + 1) << 16` in the upper 16
    # bits; 0 means ground/noise, >0 means a real cluster.
    raw = np.fromfile(path, dtype=np.uint32)
    inst = (raw >> 16).astype(np.int64) - 1  # -1 = ground/noise, >=0 = cluster
    return inst


def decode_ground_labels(path):
    raw = np.fromfile(path, dtype=np.uint32)
    return raw == GROUND_LABEL


def color_for_instances(inst_ids):
    # Stable per-cluster colors via a cyclic prism colormap; -1 stays unused.
    unique = np.unique(inst_ids[inst_ids >= 0])
    if unique.size == 0:
        return np.zeros((inst_ids.size, 3), dtype=np.uint8)
    cmap = plt.get_cmap("prism")
    palette = (cmap(np.arange(unique.size) / max(unique.size, 1))[:, :3] * 255).astype(
        np.uint8
    )
    lookup = {cid: palette[i] for i, cid in enumerate(unique)}
    out = np.zeros((inst_ids.size, 3), dtype=np.uint8)
    for cid, rgb in lookup.items():
        out[inst_ids == cid] = rgb
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument(
        "--kitti_dir",
        required=True,
        help="Dataset root, matching kitti_clustering.py's --kitti_dir",
    )
    p.add_argument(
        "--save_dir",
        default=None,
        help="Sequence directory holding patchwork/ and hdbscan/. Defaults to "
        "the sequence dir under --kitti_dir.",
    )
    p.add_argument("-s", "--seq", default="05")
    p.add_argument("-i", "--init_stamp", type=int, default=0)
    p.add_argument("-e", "--end_stamp", type=int, default=10)
    p.add_argument(
        "--save",
        default=None,
        help="Optional .rrd path to save the recording instead of spawning the "
        "viewer (e.g. cluster_viz.rrd). When set, the viewer is not spawned.",
    )
    p.add_argument(
        "--show-raw",
        action="store_true",
        help="Also log the raw (uncolored) scan as world/raw.",
    )
    args = p.parse_args()

    data_base, save_base = resolve_dirs(args.kitti_dir, args.save_dir, args.seq)
    cloud_dir = data_base + "/velodyne"
    ground_dir = save_base + "/patchwork"
    hdbscan_dir = save_base + "/hdbscan"

    rr.init("erasor2_kitti_clustering", spawn=args.save is None)
    if args.save is not None:
        rr.save(args.save)

    # Fix the world frame to a familiar orientation (z up, +x forward).
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    for i in tqdm(range(args.init_stamp, args.end_stamp + 1)):
        stem = str(i).zfill(6)
        scan_path = cloud_dir + "/" + stem + ".bin"
        ground_path = ground_dir + "/" + stem + ".label"
        hdbscan_path = hdbscan_dir + "/" + stem + ".label"

        xyz = load_scan(scan_path)
        ground_mask = decode_ground_labels(ground_path)
        inst = decode_instance_labels(hdbscan_path)

        rr.set_time("frame", sequence=i)

        if args.show_raw:
            rr.log(
                "world/raw",
                rr.Points3D(xyz, colors=np.full((xyz.shape[0], 3), 128, dtype=np.uint8)),
            )

        rr.log(
            "world/ground",
            rr.Points3D(
                xyz[ground_mask],
                colors=np.tile(np.array([139, 90, 43], dtype=np.uint8), (ground_mask.sum(), 1)),
            ),
        )

        non_ground = ~ground_mask
        inst_ng = inst[non_ground]
        valid = inst_ng >= 0
        if valid.any():
            colors = color_for_instances(inst_ng[valid])
            rr.log(
                "world/instances",
                rr.Points3D(xyz[non_ground][valid], colors=colors),
            )
        else:
            rr.log("world/instances", rr.Clear(recursive=False))


if __name__ == "__main__":
    main()

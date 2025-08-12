import argparse
import os

import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

# from tabulate import tabulate  # unused import
from tqdm import tqdm

DYNAMIC_CLASSES = [252, 253, 254, 255, 256, 257, 259]


# ---------- IO & helpers ----------
def read_pcd_o3d_tensor(path):
    """
    Read a PCD file using Open3D's Tensor API.
    Returns XYZ coordinates and intensity values as uint32.
    If intensity is stored as float, safely reinterpret or cast to uint32.
    """
    pcd = o3d.t.io.read_point_cloud(path)
    if "positions" in pcd.point:
        xyz = pcd.point["positions"].numpy()
    else:
        raise RuntimeError("PCD has no positions field.")

    if "intensity" in pcd.point:
        intens = pcd.point["intensity"].numpy().reshape(-1)
    elif "label" in pcd.point:
        # Sometimes intensity is stored under 'label'
        intens = pcd.point["label"].numpy().reshape(-1)
    else:
        # Default to zeros if intensity field is missing
        intens = np.zeros((xyz.shape[0],), dtype=np.uint32)

    # Convert intensity to uint32 (via value casting or bit reinterpretation)
    intens_u32 = to_uint32_intensity(intens)
    return xyz, intens_u32


def to_uint32_intensity(arr):
    """Convert intensity array to uint32, handling both cases.

    - If dtype is already uint32, return as is.
    - If dtype is float and values are close to integers, cast directly.
    - If dtype is float with bit-encoded values, reinterpret bits as uint32.
    """
    a = np.asarray(arr)
    if a.dtype == np.uint32:
        return a
    if a.dtype == np.float64 or a.dtype == np.float32:
        # If values are nearly integers, cast directly
        frac = np.abs(a - np.round(a))
        if np.all(frac < 1e-6):
            return np.round(a).astype(np.uint32)
        # Otherwise, reinterpret float32 bits as uint32
        a32 = a.astype(np.float32, copy=False)
        return a32.view(np.uint32)
    # Fallback: direct cast
    return a.astype(np.uint32)


# ---------- label utils ----------
def intensity2labels(intensity_u32):
    """Split uint32 intensity into semantic and instance labels.

    Returns:
        - sem_label: lower 16 bits (semantic class)
        - inst_label: upper 16 bits (instance ID)
    """
    label = intensity_u32.astype(np.uint32)
    sem_label = label & 0xFFFF
    inst_label = label >> 16
    return sem_label, inst_label


def fetch_dynamic_objects_ids(intensity_u32):
    """Print unique instance IDs for each dynamic class."""
    sem_label, inst_label = intensity2labels(intensity_u32)
    for class_id in DYNAMIC_CLASSES:
        unique_ids = np.unique(inst_label[sem_label == class_id])
        print(class_id, unique_ids)


def count_static_and_dynamic(intensity_u32, verbose=False):
    """
    Count the number of static and dynamic points based on semantic labels.
    Returns a dictionary with counts for each dynamic class and summary statistics.
    """
    NUM_CLASS_ON_MAP = {
        252: 0,
        253: 0,
        254: 0,
        255: 0,
        256: 0,
        257: 0,
        258: 0,
        259: 0,
        "dynamic": 0,
        "static": 0,
        "percentage": 0.0,
        "total": 0,
    }
    sem_label, _ = intensity2labels(intensity_u32)

    if verbose:
        print("[Debug]: total points: {}".format(sem_label.shape[0]))
    num_total = 0
    for class_id in DYNAMIC_CLASSES:
        num_class = np.count_nonzero(sem_label == class_id)
        NUM_CLASS_ON_MAP[class_id] = num_class
        num_total += num_class

    nd = num_total
    ns = int(sem_label.shape[0] - nd)
    NUM_CLASS_ON_MAP["dynamic"] = int(nd)
    NUM_CLASS_ON_MAP["static"] = int(ns)
    NUM_CLASS_ON_MAP["total"] = int(ns + nd)
    NUM_CLASS_ON_MAP["percentage"] = (float(nd) / float(ns)) * 100 if ns > 0 else 0.0

    if verbose:
        print(
            f"# Total - {ns + nd} => Static : {ns} | Dynamic: {nd} | "
            f'percentage: {NUM_CLASS_ON_MAP["percentage"]:.3f}'
        )
    return NUM_CLASS_ON_MAP


# ---------- metrics ----------
def calc_naive_preservation(
    gt_xyz, gt_int_u32, est_xyz, est_int_u32, dists, indices, voxelsize
):
    """
    Compute naive preservation metrics:
    - num_preserved: number of geometric inliers
    - num_static_preserved: number of static points preserved
    - num_dynamic_preserved: number of dynamic points preserved
    """
    num_pc = gt_xyz.shape[0]
    assert num_pc == dists.shape[0]
    assert num_pc == indices.shape[0]

    DETERMINISTIC_INLIER_THR = voxelsize * np.sqrt(3) / 2

    gt_sem_label, _ = intensity2labels(gt_int_u32)
    est_sem_label, _ = intensity2labels(est_int_u32[indices])

    is_inliers = dists < DETERMINISTIC_INLIER_THR

    num_preserved = int(np.count_nonzero(is_inliers))
    gt_inliers = gt_sem_label[is_inliers]
    est_inliers = est_sem_label[is_inliers]

    num_static_preserved = 0
    num_dynamic_preserved = 0

    for gt_sem, est_sem in tqdm(
        zip(gt_inliers, est_inliers), total=len(gt_inliers), desc="Preservation"
    ):
        if (gt_sem not in DYNAMIC_CLASSES) and (est_sem not in DYNAMIC_CLASSES):
            num_static_preserved += 1
        elif (gt_sem in DYNAMIC_CLASSES) and (est_sem in DYNAMIC_CLASSES):
            num_dynamic_preserved += 1
        else:
            pass

    return num_preserved, num_static_preserved, num_dynamic_preserved


def evaluate(gt_xyz, gt_int_u32, est_xyz, est_int_u32, voxelsize=0.2):
    """
    Evaluate static/dynamic preservation performance between GT and estimate.
    Computes preservation, rejection, and F1-score.
    """
    num_gt = count_static_and_dynamic(gt_int_u32)
    # num_estimate = count_static_and_dynamic(est_int_u32)  # noqa: F841

    nbrs = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(est_xyz)
    dists, indices = nbrs.kneighbors(gt_xyz)
    dists = dists.reshape(-1)
    indices = indices.reshape(-1)

    (
        num_preserved,
        num_static_preserved,
        num_dynamic_preserved,
    ) = calc_naive_preservation(
        gt_xyz, gt_int_u32, est_xyz, est_int_u32, dists, indices, voxelsize
    )

    # gt_data = [num_gt["static"], num_gt["dynamic"], f"{num_gt['percentage']:.3f}"]
    # est_data = [
    #     num_estimate["static"],
    #     num_estimate["dynamic"],
    #     f"{num_estimate['percentage']:.3f}",
    # ]

    # precision = float(num_gt["dynamic"] - num_estimate["dynamic"]) / float(
    #     num_gt["total"] - num_estimate["total"] + 1e-11
    # )
    # recall = float(num_gt["dynamic"] - num_estimate["dynamic"]) / float(
    #     num_gt["dynamic"] + 1e-11
    # )

    # pr = (float(num_static_preserved) /
    #       float(max(num_gt['static'], 1)) * 100.0)  # noqa: F841
    rr = (
        float(num_gt["dynamic"] - num_dynamic_preserved)
        / (float(max(num_gt["dynamic"], 1)))
        * 100.0
    )
    # f1 = 0.0
    # if (pr + rr) > 0:
    #     f1 = 2 * (pr / 100) * (rr / 100) / ((pr / 100) + (rr / 100))

    # printed_data = gt_data + est_data + [f"{pr:.3f}", f"{rr:.3f}", f"{f1:.4f}"]
    # print(tabulate([printed_data],
    #                headers=['# stat. pts', '# dyn. pts', '%',
    #                         '# est. stat. pts', '# est. dyn. pts', '%',
    #                         'Preservation', 'Rejection', 'F1'],
    #                tablefmt='orgtbl'))
    # For KETI's evaluation
    print(f"\033[1;32mFinal <removal rate> is: {rr:.3f}%\033[0m")


# ---------- main ----------
def main():
    """Main function to parse arguments and run evaluation."""
    parser = argparse.ArgumentParser(description="Analysis of static map (Open3D)")
    parser.add_argument(
        "--gt", required=True, type=str, help="Path to ground truth PCD"
    )
    parser.add_argument("--est", required=True, type=str, help="Path to estimated PCD")
    parser.add_argument(
        "--vox", default=0.2, type=float, help="Voxel size for distance threshold"
    )
    args = parser.parse_args()

    print("GT Path: " + args.gt)
    print("Estimate Path: " + args.est)

    assert os.path.isfile(args.gt), "GT path does not exist"
    assert os.path.isfile(args.est), "Estimate path does not exist"

    print("Loading point clouds...")
    gt_xyz, gt_int_u32 = read_pcd_o3d_tensor(args.gt)
    est_xyz, est_int_u32 = read_pcd_o3d_tensor(args.est)
    print("Finished loading point clouds.")

    evaluate(gt_xyz, gt_int_u32, est_xyz, est_int_u32, voxelsize=args.vox)


if __name__ == "__main__":
    main()

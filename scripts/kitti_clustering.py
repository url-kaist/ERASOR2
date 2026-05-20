import argparse
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pypatchworkpp
from pcd_preprocess import clusters_hdbscan
from tqdm import tqdm

vis = o3d.visualization.Visualizer()

"""
Use like this:

python3 kitti_clustering.py \
    --kitti_dir /home/url/datasets/kitti \
    --seq "05" --init_stamp 2350 --end_stamp 2670 \
    --save-instance-labels --save-ground-labels
"""
if __name__ == "__main__":
    version = "V2"  # 'V1': Ground is considered as '9', 'V2': Ground is '1'
    if version == "V1":
        GROUND_LABEL = 9
    else:
        GROUND_LABEL = 1

    parser = argparse.ArgumentParser(
        description="Run Patchwork++ ground segmentation + HDBSCAN instance "
        "clustering over SemanticKITTI / HeLiMOS frames."
    )
    parser.add_argument(
        "--kitti_dir",
        required=True,
        help="Dataset root. For SemanticKITTI, this is the directory ABOVE "
        "'dataset/' (e.g. /home/url/datasets/kitti, so that "
        "<kitti_dir>/dataset/sequences/<seq>/velodyne/ exists). For HeLiMOS, "
        "this is the directory directly containing the sequence subfolders.",
    )
    parser.add_argument(
        "--save_dir",
        default=None,
        help="Sequence directory to write hdbscan/ + patchwork/ into. If "
        "omitted, labels are written inside the same sequence directory that "
        "holds 'velodyne/' and 'labels/' (i.e. "
        "<kitti_dir>/dataset/sequences/<seq>/ for SemanticKITTI, or "
        "<kitti_dir>/<seq>/ for HeLiMOS).",
    )
    parser.add_argument(
        "-s",
        "--seq",
        default="Merged",
        help="sequence of the dataset. For HeLiMOS: 'Merged', "
        "For SemanticKITTI: '00', '01', '02', ..., '21'",
    )
    parser.add_argument("-i", "--init_stamp", default=0, type=int)
    parser.add_argument("-e", "--end_stamp", default=12477, type=int)
    parser.add_argument(
        "--save-ground-labels", action="store_true", help="save ground labels to file"
    )
    parser.add_argument(
        "--save-instance-labels",
        action="store_true",
        help="save instance labels to file",
    )
    parser.add_argument(
        "--use-pre-extracted-ground-label",
        action="store_true",
        help="use pre-extracted ground labels instead of pypatchworkpp",
    )
    args = parser.parse_args()

    kitti_dir = args.kitti_dir.rstrip("/")

    # Determine dataset type and resolve the per-sequence base directory that
    # holds velodyne/ (and labels/). SemanticKITTI lives under
    # <root>/dataset/sequences/<seq>/; HeLiMOS lives directly under <root>/<seq>/.
    if args.seq == "Merged":
        dataset_type = "HeLiMOS"
        data_base = kitti_dir + "/" + args.seq
    else:
        # SemanticKITTI dataset - validate sequence format
        valid_seqs = ["{:02d}".format(i) for i in range(22)]  # 00 to 21
        if args.seq not in valid_seqs:
            print(
                "Error: Invalid sequence '{}' for SemanticKITTI. "
                "Valid sequences are: {}".format(args.seq, ", ".join(valid_seqs))
            )
            exit(1)

        dataset_type = "SemanticKITTI"
        data_base = kitti_dir + "/dataset/sequences/" + args.seq

    # Default: write the new labels next to velodyne/ and labels/ inside the
    # sequence dir. If --save_dir is given, treat it as the sequence dir to
    # write into directly.
    save_base = args.save_dir.rstrip("/") if args.save_dir else data_base

    print(
        "Using {} dataset with sequence: {} (input: {}, output: {})".format(
            dataset_type, args.seq, data_base, save_base
        )
    )

    # Use argparse values
    save_ground_labels = args.save_ground_labels
    save_instance_labels = args.save_instance_labels
    use_pre_extracted_ground_label = args.use_pre_extracted_ground_label

    # Initialize Patchwork++ if not using pre-extracted ground labels
    if not use_pre_extracted_ground_label:
        params = pypatchworkpp.Parameters()
        params.verbose = False
        PatchworkPLUSPLUS = pypatchworkpp.patchworkpp(params)

    cloud_dir = data_base + "/velodyne"
    ground_label_dir = save_base + "/patchwork"
    output_dir = save_base + "/hdbscan"
    if save_ground_labels:
        Path(ground_label_dir).mkdir(parents=True, exist_ok=True)
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    for i in tqdm(range(args.init_stamp, args.end_stamp + 1)):
        zfilled_idx = str(i).zfill(6)
        pcd_path = cloud_dir + "/" + zfilled_idx + ".bin"
        scan = np.fromfile(pcd_path, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        scan_xyz = scan[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(scan_xyz)

        num_pts = np.asarray(pcd.points).shape[0]
        if use_pre_extracted_ground_label:
            ground_file = f"{ground_label_dir}/{zfilled_idx}.label"
            ground_labels = np.fromfile(ground_file, dtype=np.uint32)
            ground_labels.reshape((-1))
            ground_inliers = list(np.where(ground_labels == GROUND_LABEL)[0])
        else:
            # Use pypatchworkpp for ground segmentation
            PatchworkPLUSPLUS.estimateGround(scan)
            ground_indices = PatchworkPLUSPLUS.getGroundIndices()
            ground_inliers = list(ground_indices)

            if save_ground_labels:
                output_fname = ground_label_dir + "/" + str(i).zfill(6) + ".label"
                ground_labels = np.zeros(num_pts, dtype=np.uint32)
                ground_labels[ground_inliers] = GROUND_LABEL
                ground_labels.astype(np.uint32).tofile(output_fname)

        start_time = time.time()
        pcd_ = pcd.select_by_index(ground_inliers, invert=True)
        labels_ = np.expand_dims(clusters_hdbscan(np.asarray(pcd_.points)), axis=-1)
        # print("--- %s seconds ---" % (time.time() - start_time))
        # f = open("/home/shapelim/hdbscan_time.txt", 'a')
        # f.write("%f\n"% (time.time() - start_time))
        # f.close()

        labels = np.ones((num_pts, 1)) * -1
        mask = np.ones(num_pts, dtype=bool)
        mask[ground_inliers] = False

        # Only non-ground points are updated
        labels[mask] = labels_

        # VISUALIZATION
        # Note: -1 in pred indicates sub-cluster and ground points
        points = pcd.points
        pred = labels.reshape(-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        colors_pred = np.zeros((len(pred), 4))
        flat_indices = np.unique(pred)
        max_instance = len(flat_indices)
        colors_instance = plt.get_cmap("prism")(
            np.arange(len(flat_indices)) / (max_instance if max_instance > 0 else 1)
        )

        for idx in range(len(flat_indices)):
            colors_pred[pred == idx] = colors_instance[int(idx)]

        colors_pred[pred == -1] = [0.0, 0.0, 0.0, 0.0]

        pcd.colors = o3d.utility.Vector3dVector(colors_pred[:, :3])

        ##################
        # Save label files
        # It follows SemanticKITTI labels
        # Finally, 0: ground + sub-clustered points
        # > 0: instances
        if save_instance_labels:
            output_fname = output_dir + "/" + str(i).zfill(6) + ".label"
            sem = np.zeros_like(pred).astype(np.float32)
            # NOTE(hlim): Ground points in label file are assigned to -1
            ins = pred.astype(int) + 1
            pred_eval = sem + (ins << 16)
            pred_eval.astype(np.uint32).tofile(output_fname)
        else:
            o3d.visualization.draw_geometries([pcd])
        ##################

        # o3d.visualization.draw_geometries([pcd])

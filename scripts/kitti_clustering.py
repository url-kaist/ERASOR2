import time
import argparse
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from pcd_preprocess import clusters_hdbscan
from pathlib import Path

vis = o3d.visualization.Visualizer()
if __name__ == "__main__":
    save_ground_labels = False
    save_instance_labels = True
    use_pre_extracted_ground_label = True
    version = "V2" # "V1": Ground is considered as '9', "V2": Ground is considered as '1'
    if (version == "V1"):
        GROUND_LABEL = 9
    else:
        GROUND_LABEL = 1

    odometry_sequences = []
    for s in range(22):
        odometry_sequences.append(str(s).zfill(2))

    ABS_DATA_DIR = "/home/shapelim/Documents/KAIST05/deskewed_LiDAR"
    ABS_SAVE_DIR = "/home/shapelim/Documents/KAIST05/deskewed_LiDAR"

    parser = argparse.ArgumentParser(description="Convert KITTI dataset to ROS bag file the easy way!")
    # parser.add_argument("--dir", nargs="?", default = os.getcwd(), help="base directory of the dataset, if no directory passed the deafult is current working directory")
    parser.add_argument("-s", "--seq", default="Merged", help="sequence of the odometry dataset (between 00 - 21), option is only for ODOMETRY datasets.")
    parser.add_argument("-i", "--init_stamp", default=9000, type=int)
    parser.add_argument("-e", "--end_stamp", default=12477, type=int)
    args = parser.parse_args()

    cloud_dir = ABS_DATA_DIR + "/" + args.seq + "/velodyne"
    ground_label_dir = ABS_DATA_DIR + "/" + args.seq + "/patchwork"
    output_dir = ABS_SAVE_DIR + "/" + args.seq + "/hdbscan"
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    for i in range(args.init_stamp, args.end_stamp + 1):
        zfilled_idx = str(i).zfill(6)
        pcd_path = cloud_dir + "/" + zfilled_idx + ".bin"
        scan = np.fromfile(pcd_path, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        scan_xyz = scan[:, :3]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(scan_xyz)

        num_pts = np.asarray(pcd.points).shape[0]
        if (use_pre_extracted_ground_label):
            ground_file = f'{ground_label_dir}/{zfilled_idx}.label'
            ground_labels = np.fromfile(ground_file, dtype=np.uint32)
            ground_labels.reshape((-1))
            ground_inliers = list(np.where(ground_labels == GROUND_LABEL)[0])

        ##################
        # if (save_ground_labels):
        #     output_fname = abs_ground_dir + "/" + str(i).zfill(6) + ".label"
        #     ground_labels = np.zeros(num_pts, dtype=np.uint32)
        #     ground_labels[ground_inliers] = GROUND_LABEL
        #     ground_labels.astype(np.uint32).tofile(output_fname)
        ##################
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

        ############ VISUALIZATION ############
        # Note that -1 in pred indicates the points are sub-cluster and ground points
        points = pcd.points
        pred = labels.reshape(-1)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        colors_pred = np.zeros((len(pred), 4))
        flat_indices = np.unique(pred)
        max_instance = len(flat_indices)
        colors_instance = plt.get_cmap("prism")(np.arange(len(flat_indices)) / (max_instance if max_instance > 0 else 1))

        for idx in range(len(flat_indices)):
            colors_pred[pred == idx] = colors_instance[int(idx)]

        colors_pred[pred == -1] = [0., 0., 0., 0.]

        pcd.colors = o3d.utility.Vector3dVector(colors_pred[:, :3])

        ##################
        # Save label files
        # It follows SemanticKITTI labels
        # Finally, 0: ground + sub-clustered points
        # > 0: instances
        if (save_instance_labels):
            output_fname = output_dir + "/" + str(i).zfill(6) + ".label"
            sem = np.zeros_like(pred).astype(np.float32)
            ins = pred.astype(int) + 1
            pred_eval = sem + (ins << 16)
            pred_eval.astype(np.uint32).tofile(output_fname)
        else:
            o3d.visualization.draw_geometries([pcd])
        ##################

        # o3d.visualization.draw_geometries([pcd])

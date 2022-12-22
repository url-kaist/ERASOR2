import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from pcd_preprocess import clusters_hdbscan

def concentric_zone_based_ground_segmentation(points, range_thrs, height_thrs):
    assert(range_thrs[-1] == float('inf') and len(range_thrs) == len(height_thrs))
    num_thrs = len(height_thrs)
    points_np = np.asarray(points)
    indices = np.arange(points_np.shape[0])
    range_wrt_origin = np.sqrt(np.square(points_np[:, 0]) + np.square(points_np[:, 1]))
    inliers = []
    for i in range(num_thrs):
        if i == 0:
            range_within = range_wrt_origin < range_thrs[i]
        else:
            range_within = np.bitwise_and(range_thrs[i - 1] < range_wrt_origin, range_wrt_origin < range_thrs[i])

        mask = points_np[range_within, 2] < height_thrs[i] # true -> ground
        inliers_per_zone = list(indices[range_within][mask])
        inliers = inliers + inliers_per_zone
    return inliers

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

    abs_ground_dir = "/media/shapelim/UX980/erasor_inputs/bongeunsa_dataset/ground_labels_via_patchwork2"
    abs_instance_dir = "/media/shapelim/UX980/erasor_inputs/bongeunsa_dataset/instance_labels_via_patchwork2"

    for i in range(0, 1320):
        zfilled_idx = str(i).zfill(6)
        pcd_path = "/media/shapelim/UX980/erasor_inputs/bongeunsa_dataset/pcds/" + zfilled_idx + ".pcd"
        pcd = o3d.io.read_point_cloud(pcd_path)

        num_pts = np.asarray(pcd.points).shape[0]
        if (use_pre_extracted_ground_label):
            ground_file = f'/media/shapelim/UX980/erasor_inputs/bongeunsa_dataset/ground_labels_via_patchwork2/{zfilled_idx}.label'
            ground_labels = np.fromfile(ground_file, dtype=np.uint32)
            ground_labels.reshape((-1))
            ground_inliers = list(np.where(ground_labels == GROUND_LABEL)[0])
        else:
            ground_inliers = concentric_zone_based_ground_segmentation(pcd.points, [4.0, 10.0, float('inf')], [-1.35, -1.2, -1.0])

        ##################
        # if (save_ground_labels):
        #     output_fname = abs_ground_dir + "/" + str(i).zfill(6) + ".label"
        #     ground_labels = np.zeros(num_pts, dtype=np.uint32)
        #     ground_labels[ground_inliers] = GROUND_LABEL
        #     ground_labels.astype(np.uint32).tofile(output_fname)
        ##################

        pcd_ = pcd.select_by_index(ground_inliers, invert=True)
        labels_ = np.expand_dims(clusters_hdbscan(np.asarray(pcd_.points)), axis=-1)

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
            output_fname = abs_instance_dir + "/" + str(i).zfill(6) + ".label"
            sem = np.zeros_like(pred).astype(np.float32)
            ins = pred.astype(int) + 1
            pred_eval = sem + (ins << 16)
            pred_eval.astype(np.uint32).tofile(output_fname)
        else:
            o3d.visualization.draw_geometries([pcd])
        ##################

        # o3d.visualization.draw_geometries([pcd])

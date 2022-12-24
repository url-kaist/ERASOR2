import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from pcd_preprocess import clusters_hdbscan
from pathlib import Path

if __name__ == "__main__":
    abs_src_dir = "/media/shapelim/Elements/3DUIS"
    abs_save_dir = "/media/shapelim/Elements/SemanticKITTI_w_gt/dataset/sequences"
    seq = "07"
    start_frame = 630
    end_frame = 820 + 1

    output_dir = abs_save_dir + "/" + seq + "/cais"
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    for i in range(start_frame, end_frame):
        zfilled_idx = str(i).zfill(6)
        npy_path = abs_src_dir + "/" + seq + "/raw_pred/" + zfilled_idx + ".npy"
        pred = np.load(npy_path)
        points = pred[:, :3]
        instance_label = pred[:, -1]

        save_path = output_dir + "/" + zfilled_idx + ".label"
        print("Save path: ", save_path)
        sem = np.zeros_like(instance_label).astype(np.float32)
        ins = instance_label.astype(int)
        pred_eval = sem + (ins << 16)
        pred_eval.astype(np.uint32).tofile(save_path)
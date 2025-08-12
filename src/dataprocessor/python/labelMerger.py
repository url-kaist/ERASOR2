"""
label merging from Patchwork and ERASOR2!
Patchwork: 0 (non-ground), 1 (ground)
ERASOR2: 0 (Static), 251 (Moving)

=> Merged label: 0 (Static), 9 (Ground), 251 (Moving)
"""
import os

import numpy as np

# import open3d as o3d  # noqa: F401
# import yaml  # noqa: F401
from tqdm import tqdm


def load_labels(label_path):
    """Load semantic and instance labels from file."""
    label = np.fromfile(label_path, dtype=np.uint32)
    label = label.reshape((-1))

    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16  # instance id in upper half

    # sanity check
    assert (sem_label + (inst_label << 16) == label).all()

    return sem_label, inst_label


def encode_label(sem_label, inst_label):
    """Encode semantic and instance labels into single uint32."""
    return np.bitwise_or(
        sem_label.astype(np.uint32), np.left_shift(inst_label.astype(np.uint32), 16)
    )


def rearrange_inst_label(inst_label, cnt):
    """Rearrange instance labels starting from cnt."""
    new_label = np.copy(inst_label)
    unique_labels = np.unique(inst_label)

    for label in unique_labels:
        if label == 0:
            continue
        else:
            new_label[inst_label == label] = cnt
            cnt += 1

    return new_label


if __name__ == "__main__":
    seq = "Merged"  # Set as 'Merged'
    home_dir = os.path.expanduser("~")
    patchwork_root = (
        home_dir
        + "/kmos_bench/sequences/KAIST05/deskewed_LiDAR/{}/patchwork".format(seq)
    )
    mos_root = home_dir + "/kmos_bench/sequences/KAIST05/deskewed_LiDAR/{}/mos".format(
        seq
    )
    hdbscan_root = (
        home_dir + "/kmos_bench/sequences/KAIST05/deskewed_LiDAR/{}/hdbscan".format(seq)
    )
    new_label_root = (
        home_dir + "/kmos_bench/sequences/KAIST05/deskewed_LiDAR/{}/labels".format(seq)
    )

    if not os.path.exists(new_label_root):
        os.makedirs(new_label_root)

    label_names = os.listdir(patchwork_root)
    label_names.sort()

    cnt = 1
    for i in tqdm(range(199), desc="Processing labels"):
        ground_label, _ = load_labels(os.path.join(patchwork_root, label_names[i]))
        _, hdbscan_label = load_labels(os.path.join(hdbscan_root, label_names[i]))
        # print(np.unique(hdbscan_label))
        mos_label, _ = load_labels(os.path.join(mos_root, label_names[i]))
        # print(np.unique(mos_label))
        mos_mask = mos_label > 250
        inv_mos_mask = np.logical_not(mos_mask)
        ground_mask = ground_label > 0
        ground_mos_label = np.copy(ground_label)

        ground_mos_label[mos_mask] = 251
        ground_mos_label[ground_mask] = 9

        # new_inst_label = rearrange_inst_label(hdbscan_label, cnt)  # noqa: F841
        new_inst_label = np.zeros(hdbscan_label.shape, dtype=np.uint32)
        new_inst_label[inv_mos_mask] = 0

        encoded_label = encode_label(ground_mos_label, new_inst_label)
        encoded_label = encoded_label.reshape((-1))

        sem_label = encoded_label & 0xFFFF  # semantic label in lower half
        inst_label = encoded_label >> 16  # instance id in upper half

        encoded_label.tofile(os.path.join(new_label_root, label_names[i]))

import numpy as np
from tabulate import tabulate
from time import time
import os
import open3d as o3d
import math
from tqdm import tqdm

# ------------------------ CONFIG ------------------------
SEQ_NAME = "00"
RAW_BIN_PATH = f"/home/suba/my_ros2_ws/src/my_voxel_filter_pkg/00/velodyne"
LABEL_PATH = f"/home/suba/my_ros2_ws/src/my_voxel_filter_pkg/data_odometry_labels/dataset/sequences/00/labels"
FILTERED_PCD_PATH = f"/home/suba/voxel_map_debug_2.0"
ALGORITHM_NAME = "voxel_filter"
# --------------------------------------------------------

# SemanticKITTI dynamic class labels
DYNAMIC_CLASSES = {1, 6, 7, 8, 9, 10, 11, 13, 14, 15, 18, 20}

def is_dynamic(label):
    return (label & 0xFFFF) in DYNAMIC_CLASSES

def classify_labels(labels):
    labels = labels & 0xFFFF
    dynamic_mask = np.isin(labels, list(DYNAMIC_CLASSES))
    return {
        'static': np.count_nonzero(~dynamic_mask),
        'dynamic': np.count_nonzero(dynamic_mask),
        'mask': dynamic_mask
    }

def load_bin_points(bin_path):
    return np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)

def load_labels(label_path):
    return np.fromfile(label_path, dtype=np.uint32)

def load_pcd_points(pcd_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    return np.asarray(pcd.points)

def match_filtered_to_raw(filtered_points, raw_points):
    # Brute force nearest neighbor matching (can optimize with KD-tree)
    from scipy.spatial import cKDTree
    tree = cKDTree(raw_points[:, :3])
    _, idxs = tree.query(filtered_points, k=1)
    matched_mask = np.zeros(len(raw_points), dtype=bool)
    matched_mask[idxs] = True
    return matched_mask

def evaluate_sequence():
    printed_data = []
    total_gt_static = 0
    total_gt_dynamic = 0
    total_correct_static = 0
    total_correct_dynamic = 0
    total_pred_static = 0
    total_pred_dynamic = 0

    # Skip index 0 to match 4540 filtered outputs
    bin_files = sorted(os.listdir(RAW_BIN_PATH))[1:]
    label_files = sorted(os.listdir(LABEL_PATH))[1:]
    pcd_files = sorted(os.listdir(FILTERED_PCD_PATH))

    for i, (bin_name, label_name, pcd_name) in enumerate(tqdm(zip(bin_files, label_files, pcd_files), total=4540)):
        bin_path = os.path.join(RAW_BIN_PATH, bin_name)
        label_path = os.path.join(LABEL_PATH, label_name)
        pcd_path = os.path.join(FILTERED_PCD_PATH, pcd_name)

        raw_points = load_bin_points(bin_path)
        raw_labels = load_labels(label_path)
        filtered_points = load_pcd_points(pcd_path)

        if len(filtered_points) == 0:
            continue

        gt_info = classify_labels(raw_labels)
        matched_mask = match_filtered_to_raw(filtered_points, raw_points)

        pred_labels = raw_labels[matched_mask]
        pred_info = classify_labels(pred_labels)

        correct_static = np.count_nonzero((~gt_info['mask']) & matched_mask)
        correct_dynamic = np.count_nonzero((gt_info['mask']) & matched_mask)

        total_gt_static += gt_info['static']
        total_gt_dynamic += gt_info['dynamic']
        total_pred_static += pred_info['static']
        total_pred_dynamic += pred_info['dynamic']
        total_correct_static += correct_static
        total_correct_dynamic += correct_dynamic

    SA = 100 * total_correct_static / total_gt_static
    DA = 100 * total_correct_dynamic / total_gt_dynamic
    AA = math.sqrt(SA * DA)   #Arithemtic Accuracy - geometry mean of SA and DA
    HA = 2 * SA * DA / (SA + DA)  #Harmonic Accuracy - harmonic mean of SA and DA

    printed_data.append([
        ALGORITHM_NAME,
        total_pred_static,
        total_pred_dynamic,
        SA, DA, AA, HA
    ])

    print(f"\nEvaluation results in seq \033[92m{SEQ_NAME}\033[0m")
    print(tabulate(printed_data, headers=[
        'Methods', '# static', '# dynamics', 'SA [%] ↑', 'DA [%] ↑', 'AA [%] ↑', 'HA [%] ↑'
    ], tablefmt='orgtbl'))

if __name__ == "__main__":
    t0 = time()
    evaluate_sequence()
    print(f"Time cost: {(time() - t0):.2f}s")

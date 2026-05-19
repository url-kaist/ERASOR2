#!/usr/bin/env python3
"""Soft-check ERASOR2 PR / RR / F1 against tolerance thresholds.

Used by the CI parity workflow after the strict cmp passes. The metric
logic mirrors scripts/evaluate.py.
"""
import argparse
import sys

import numpy as np
import open3d as o3d
from sklearn.neighbors import NearestNeighbors

DYNAMIC_CLASSES = [252, 253, 254, 255, 256, 257, 259]


def read_pcd(path):
    pcd = o3d.t.io.read_point_cloud(path)
    xyz = pcd.point["positions"].numpy()
    if "intensity" in pcd.point:
        raw = pcd.point["intensity"].numpy().reshape(-1)
    elif "label" in pcd.point:
        raw = pcd.point["label"].numpy().reshape(-1)
    else:
        raw = np.zeros((xyz.shape[0],), dtype=np.uint32)
    a = np.asarray(raw)
    if a.dtype == np.uint32:
        intens = a
    elif a.dtype in (np.float32, np.float64):
        frac = np.abs(a - np.round(a))
        if np.all(frac < 1e-6):
            intens = np.round(a).astype(np.uint32)
        else:
            intens = a.astype(np.float32, copy=False).view(np.uint32)
    else:
        intens = a.astype(np.uint32)
    return xyz, intens


def semantic_label(intensity_u32):
    return intensity_u32.astype(np.uint32) & 0xFFFF


def count_static_dynamic(intensity_u32):
    sem = semantic_label(intensity_u32)
    n_dyn = int(sum(int((sem == c).sum()) for c in DYNAMIC_CLASSES))
    return {"static": int(sem.shape[0]) - n_dyn, "dynamic": n_dyn}


def compute_pr_rr(gt_xyz, gt_i, est_xyz, est_i, voxel=0.2):
    nbrs = NearestNeighbors(n_neighbors=1, algorithm="kd_tree").fit(est_xyz)
    dists, idx = nbrs.kneighbors(gt_xyz)
    dists = dists.reshape(-1)
    idx = idx.reshape(-1)
    inlier_thr = voxel * np.sqrt(3) / 2
    inliers = dists < inlier_thr
    gt_sem = semantic_label(gt_i)[inliers]
    est_sem = semantic_label(est_i[idx])[inliers]
    gt_is_dyn = np.isin(gt_sem, DYNAMIC_CLASSES)
    est_is_dyn = np.isin(est_sem, DYNAMIC_CLASSES)
    n_sp = int(((~gt_is_dyn) & (~est_is_dyn)).sum())
    n_dp = int((gt_is_dyn & est_is_dyn).sum())
    n_gt = count_static_dynamic(gt_i)
    pr = float(n_sp) / max(n_gt["static"], 1) * 100.0
    rr = float(n_gt["dynamic"] - n_dp) / max(n_gt["dynamic"], 1) * 100.0
    f1 = (
        2 * (pr / 100) * (rr / 100) / ((pr / 100) + (rr / 100))
        if (pr + rr) > 0
        else 0.0
    )
    return pr, rr, f1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gt", required=True)
    p.add_argument("--est", required=True)
    p.add_argument("--voxel", type=float, default=0.2)
    p.add_argument("--pr-min", type=float, default=97.5)
    p.add_argument("--rr-min", type=float, default=98.0)
    p.add_argument("--f1-min", type=float, default=0.97)
    args = p.parse_args()

    gt_xyz, gt_i = read_pcd(args.gt)
    est_xyz, est_i = read_pcd(args.est)
    pr, rr, f1 = compute_pr_rr(gt_xyz, gt_i, est_xyz, est_i, args.voxel)
    print("PR={:.3f}%  RR={:.3f}%  F1={:.4f}".format(pr, rr, f1))

    failures = []
    if pr < args.pr_min:
        failures.append("PR {:.3f} < {}".format(pr, args.pr_min))
    if rr < args.rr_min:
        failures.append("RR {:.3f} < {}".format(rr, args.rr_min))
    if f1 < args.f1_min:
        failures.append("F1 {:.4f} < {}".format(f1, args.f1_min))
    if failures:
        print("FAIL: " + "; ".join(failures), file=sys.stderr)
        sys.exit(1)
    print("OK")


if __name__ == "__main__":
    main()

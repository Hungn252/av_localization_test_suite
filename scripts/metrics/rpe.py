#!/usr/bin/env python3
"""
Relative Pose Error (RPE).

Reads two TUM-format files (timestamp tx ty tz qx qy qz qw), associates by
nearest timestamp, then computes translational and rotational RPE at a fixed
distance step (default 1 m) along the GT trajectory.

Usage:
    python3 rpe.py <gt_tum> <est_tum> [--delta 1.0] [--max_diff 0.02] [--output metrics.json]
"""

import argparse
import json
import sys

import numpy as np


# ── Quaternion / SE3 helpers ──────────────────────────────────────────────────

def quat_to_rot(qx, qy, qz, qw):
    """Unit quaternion (x,y,z,w) → 3×3 rotation matrix."""
    s = 2.0 / (qx*qx + qy*qy + qz*qz + qw*qw)
    return np.array([
        [1 - s*(qy*qy + qz*qz),     s*(qx*qy - qw*qz),     s*(qx*qz + qw*qy)],
        [    s*(qx*qy + qw*qz), 1 - s*(qx*qx + qz*qz),     s*(qy*qz - qw*qx)],
        [    s*(qx*qz - qw*qy),     s*(qy*qz + qw*qx), 1 - s*(qx*qx + qy*qy)],
    ])


def make_T(tx, ty, tz, qx, qy, qz, qw):
    """TUM pose components → 4×4 SE3 matrix."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3,  3] = [tx, ty, tz]
    return T


def rot_angle(R):
    """Rotation angle (rad) from a 3×3 rotation matrix."""
    cos = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cos))


# ── I/O ───────────────────────────────────────────────────────────────────────

def load_tum(path):
    """Return list of (timestamp, 4×4 SE3 matrix)."""
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            v = list(map(float, line.split()))
            poses.append((v[0], make_T(*v[1:])))
    if not poses:
        sys.exit(f'Empty file: {path}')
    return poses


# ── Timestamp association ─────────────────────────────────────────────────────

def associate(poses_gt, poses_est, max_diff):
    ts_gt = np.array([p[0] for p in poses_gt])
    candidates = {}
    for i, (t, _) in enumerate(poses_est):
        j = int(np.argmin(np.abs(ts_gt - t)))
        dt = abs(ts_gt[j] - t)
        if dt <= max_diff:
            if j not in candidates or dt < candidates[j][0]:
                candidates[j] = (dt, i)
    return [(j, candidates[j][1]) for j in sorted(candidates)]


# ── Trajectory distance ───────────────────────────────────────────────────────

def cumulative_distances(T_list):
    """Cumulative Euclidean distances along a list of 4×4 SE3 matrices."""
    dists = [0.0]
    for k in range(1, len(T_list)):
        d = np.linalg.norm(T_list[k][:3, 3] - T_list[k-1][:3, 3])
        dists.append(dists[-1] + d)
    return np.array(dists)


# ── Main ─────────────────────────────────────────────────────────────────────

def compute(gt_path, est_path, delta=1.0, max_diff=0.02):
    poses_gt  = load_tum(gt_path)
    poses_est = load_tum(est_path)

    pairs = associate(poses_gt, poses_est, max_diff)
    if len(pairs) < 2:
        sys.exit(f'Too few associated pairs ({len(pairs)}). Check timestamps or --max_diff.')

    T_gt  = [poses_gt[j][1]  for j, _ in pairs]
    T_est = [poses_est[i][1] for _, i in pairs]

    dists = cumulative_distances(T_gt)
    total_dist = dists[-1]

    trans_errs = []
    rot_errs   = []

    i = 0
    while i < len(T_gt):
        target = dists[i] + delta
        j = int(np.searchsorted(dists, target))
        if j >= len(T_gt):
            break

        # Relative transforms
        Q_gt  = np.linalg.inv(T_gt[i])  @ T_gt[j]
        Q_est = np.linalg.inv(T_est[i]) @ T_est[j]
        E     = np.linalg.inv(Q_gt) @ Q_est   # pose error

        trans_errs.append(float(np.linalg.norm(E[:3, 3])))
        rot_errs.append(rot_angle(E[:3, :3]))

        i = j   # non-overlapping segments

    if not trans_errs:
        sys.exit('No RPE segments — is the trajectory shorter than --delta?')

    te = np.array(trans_errs)
    re = np.degrees(np.array(rot_errs))

    return {
        'n_segments':      int(len(te)),
        'delta_m':         float(delta),
        'total_dist_m':    float(total_dist),
        'trans_rmse':      float(np.sqrt(np.mean(te ** 2))),
        'trans_mean':      float(np.mean(te)),
        'trans_max':       float(np.max(te)),
        'trans_std':       float(np.std(te)),
        'trans_median':    float(np.median(te)),
        'rot_rmse_deg':    float(np.sqrt(np.mean(re ** 2))),
        'rot_mean_deg':    float(np.mean(re)),
        'rot_max_deg':     float(np.max(re)),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gt',          help='Ground truth TUM file')
    ap.add_argument('est',         help='Estimated trajectory TUM file')
    ap.add_argument('--delta',     type=float, default=1.0,
                    help='Distance step between RPE pairs in metres (default: 1.0)')
    ap.add_argument('--max_diff',  type=float, default=0.02,
                    help='Max timestamp difference for association in seconds (default: 0.02)')
    ap.add_argument('--output',    help='Save metrics as JSON to this path')
    args = ap.parse_args()

    m = compute(args.gt, args.est, delta=args.delta, max_diff=args.max_diff)

    w = 18
    print(f"\n{'Metric':<{w}}  {'Value':>12}")
    print('─' * (w + 15))
    print(f"{'n_segments':<{w}}  {m['n_segments']:>12d}")
    print(f"{'delta':<{w}}  {m['delta_m']:>11.1f} m")
    print(f"{'total distance':<{w}}  {m['total_dist_m']:>11.1f} m")
    print(f"{'trans RMSE':<{w}}  {m['trans_rmse']:>11.4f} m")
    print(f"{'trans mean':<{w}}  {m['trans_mean']:>11.4f} m")
    print(f"{'trans max':<{w}}  {m['trans_max']:>11.4f} m")
    print(f"{'trans std':<{w}}  {m['trans_std']:>11.4f} m")
    print(f"{'trans median':<{w}}  {m['trans_median']:>11.4f} m")
    print(f"{'rot RMSE':<{w}}  {m['rot_rmse_deg']:>11.4f} deg")
    print(f"{'rot mean':<{w}}  {m['rot_mean_deg']:>11.4f} deg")
    print(f"{'rot max':<{w}}  {m['rot_max_deg']:>11.4f} deg")

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(m, f, indent=2)
        print(f'\nSaved → {args.output}')

    return m


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Absolute Trajectory Error (ATE).

Reads two TUM-format files (timestamp tx ty tz qx qy qz qw), associates by
nearest timestamp, applies Horn SE3 alignment, and reports translation errors.

Usage:
    python3 ate.py <gt_tum> <est_tum> [--no_align] [--max_diff 0.02] [--output metrics.json]
"""

import argparse
import json
import sys

import numpy as np


# ── I/O ───────────────────────────────────────────────────────────────────────

def load_tum(path):
    """Return (timestamps, positions) as (N,) and (N,3) arrays."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            rows.append(list(map(float, line.split())))
    if not rows:
        sys.exit(f'Empty file: {path}')
    arr = np.array(rows)
    return arr[:, 0], arr[:, 1:4]


# ── Timestamp association ─────────────────────────────────────────────────────

def associate(ts_gt, ts_est, max_diff):
    """Return list of (gt_idx, est_idx) matched by nearest timestamp."""
    # For each est frame, find closest GT frame
    candidates = {}
    for i, t in enumerate(ts_est):
        j = int(np.argmin(np.abs(ts_gt - t)))
        dt = abs(ts_gt[j] - t)
        if dt <= max_diff:
            # Keep only the closest est match per GT index
            if j not in candidates or dt < candidates[j][0]:
                candidates[j] = (dt, i)
    return [(j, candidates[j][1]) for j in sorted(candidates)]


# ── Horn SE3 alignment ────────────────────────────────────────────────────────

def align(pos_gt, pos_est):
    """
    Find R, t minimising sum ||R @ p_est_i + t - p_gt_i||^2.
    Uses the SVD-based closed-form solution (Horn 1987).
    Returns R (3,3), t (3,).
    """
    mu_gt  = pos_gt.mean(axis=0)
    mu_est = pos_est.mean(axis=0)
    A = pos_est - mu_est   # centred estimated positions (N, 3)
    B = pos_gt  - mu_gt    # centred GT positions        (N, 3)

    W = B.T @ A            # cross-covariance (3, 3)
    U, _, Vt = np.linalg.svd(W)

    # Correct for reflection
    d = np.linalg.det(U @ Vt)
    D = np.diag([1.0, 1.0, d])
    R = U @ D @ Vt
    t = mu_gt - R @ mu_est
    return R, t


# ── Main ─────────────────────────────────────────────────────────────────────

def compute(gt_path, est_path, do_align=True, max_diff=0.02):
    ts_gt,  pos_gt  = load_tum(gt_path)
    ts_est, pos_est = load_tum(est_path)

    pairs = associate(ts_gt, ts_est, max_diff)
    if len(pairs) < 3:
        sys.exit(f'Too few associated pairs ({len(pairs)}). Check timestamps or --max_diff.')

    idx_gt  = np.array([p[0] for p in pairs])
    idx_est = np.array([p[1] for p in pairs])
    P_gt    = pos_gt[idx_gt]
    P_est   = pos_est[idx_est]

    if do_align:
        R, t = align(P_gt, P_est)
        P_est = (R @ P_est.T).T + t

    errors = np.linalg.norm(P_gt - P_est, axis=1)
    return {
        'n_pairs':    int(len(pairs)),
        'ate_rmse':   float(np.sqrt(np.mean(errors ** 2))),
        'ate_mean':   float(np.mean(errors)),
        'ate_max':    float(np.max(errors)),
        'ate_std':    float(np.std(errors)),
        'ate_median': float(np.median(errors)),
        'aligned':    do_align,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gt',          help='Ground truth TUM file')
    ap.add_argument('est',         help='Estimated trajectory TUM file')
    ap.add_argument('--no_align',  action='store_true',
                    help='Skip SE3 alignment (use raw coordinates)')
    ap.add_argument('--max_diff',  type=float, default=0.02,
                    help='Max timestamp difference for association in seconds (default: 0.02)')
    ap.add_argument('--output',    help='Save metrics as JSON to this path')
    args = ap.parse_args()

    m = compute(args.gt, args.est, do_align=not args.no_align, max_diff=args.max_diff)

    w = 12
    print(f"\n{'Metric':<{w}}  {'Value':>12}")
    print('─' * (w + 15))
    print(f"{'n_pairs':<{w}}  {m['n_pairs']:>12d}")
    print(f"{'aligned':<{w}}  {str(m['aligned']):>12}")
    print(f"{'ATE RMSE':<{w}}  {m['ate_rmse']:>11.4f} m")
    print(f"{'ATE mean':<{w}}  {m['ate_mean']:>11.4f} m")
    print(f"{'ATE max':<{w}}  {m['ate_max']:>11.4f} m")
    print(f"{'ATE std':<{w}}  {m['ate_std']:>11.4f} m")
    print(f"{'ATE median':<{w}}  {m['ate_median']:>11.4f} m")

    if args.output:
        with open(args.output, 'w') as f:
            json.dump(m, f, indent=2)
        print(f'\nSaved → {args.output}')

    return m


if __name__ == '__main__':
    main()

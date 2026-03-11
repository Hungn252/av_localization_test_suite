#!/usr/bin/env python3
"""
Trajectory and error plots — equivalent to evo's output.

Produces up to six PNGs in <output_dir>:
  traj.png      — 3D trajectory overlay (GT dashed, estimated solid)
  ate_map.png   — 3D trajectory coloured by per-pose ATE (GT dashed reference)
  ate_raw.png   — ATE error over time with RMSE / mean / median / ±std band
  rpe.png       — RPE translational error per segment over distance travelled
  xyz.png       — x, y, z components vs time (GT dashed, estimated solid)
  speed.png     — estimated speed (m/s) vs time

Usage:
    python3 plot.py <gt_tum> <est_tum> <output_dir>
                    [--ate_json ate.json] [--rpe_json rpe.json]
                    [--max_diff 0.02] [--no_align] [--delta 1.0]
                    [--title "My Run"]
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np


# ── Style — matches evo's light periwinkle look ───────────────────────────────

BG      = '#E8EBF4'
GRID    = 'white'
GT_C    = '#808080'
EST_C   = '#4C72B0'
ERR_CM  = 'jet'

def _style(ax, grid_alpha=0.6):
    ax.set_facecolor(BG)
    ax.figure.set_facecolor(BG)
    ax.grid(True, color=GRID, linewidth=0.8, alpha=grid_alpha)
    for spine in ax.spines.values():
        spine.set_visible(False)

def _style3d(ax):
    ax.set_facecolor(BG)
    ax.figure.set_facecolor(BG)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor(GRID)
    ax.yaxis.pane.set_edgecolor(GRID)
    ax.zaxis.pane.set_edgecolor(GRID)
    ax.grid(True, color=GRID, linewidth=0.6)


# ── Shared helpers (inlined to keep the script self-contained) ─────────────────

def _load_tum_full(path):
    """Return (timestamps, positions (N,3), quaternions (N,4 xyzw))."""
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
    return arr[:, 0], arr[:, 1:4], arr[:, 4:8]


def _associate(ts_gt, ts_est, max_diff):
    candidates = {}
    for i, t in enumerate(ts_est):
        j = int(np.argmin(np.abs(ts_gt - t)))
        dt = abs(ts_gt[j] - t)
        if dt <= max_diff:
            if j not in candidates or dt < candidates[j][0]:
                candidates[j] = (dt, i)
    return [(j, candidates[j][1]) for j in sorted(candidates)]


def _align(pos_gt, pos_est):
    mu_gt, mu_est = pos_gt.mean(0), pos_est.mean(0)
    W = (pos_gt - mu_gt).T @ (pos_est - mu_est)
    U, _, Vt = np.linalg.svd(W)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    R = U @ D @ Vt
    t = mu_gt - R @ mu_est
    return R, t


def _quat_to_rot(q):
    """(N,4) xyzw → (N,3,3)."""
    qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    s = 2.0 / (qx**2 + qy**2 + qz**2 + qw**2)
    R = np.stack([
        np.stack([1-s*(qy**2+qz**2),   s*(qx*qy-qw*qz),   s*(qx*qz+qw*qy)], axis=1),
        np.stack([  s*(qx*qy+qw*qz), 1-s*(qx**2+qz**2),   s*(qy*qz-qw*qx)], axis=1),
        np.stack([  s*(qx*qz-qw*qy),   s*(qy*qz+qw*qx), 1-s*(qx**2+qy**2)], axis=1),
    ], axis=1)
    return R


def _cum_dist(pos):
    d = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


# ── Individual plots ──────────────────────────────────────────────────────────

def plot_traj(pos_gt, pos_est_aligned, out_path, title=''):
    fig = plt.figure(figsize=(9, 8))
    fig.patch.set_facecolor(BG)
    ax = fig.add_subplot(111, projection='3d')
    _style3d(ax)

    ax.plot(*pos_gt.T,          color=GT_C,  lw=1.2, ls='--', label='reference')
    ax.plot(*pos_est_aligned.T, color=EST_C, lw=1.2, ls='-',  label='estimated')

    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    ax.legend(fontsize=9)
    if title:
        ax.set_title(title, fontsize=10)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_ate_map(pos_gt, pos_est_aligned, errors, out_path, title=''):
    fig = plt.figure(figsize=(10, 8))
    fig.patch.set_facecolor(BG)
    ax = fig.add_subplot(111, projection='3d')
    _style3d(ax)

    # GT reference
    ax.plot(*pos_gt.T, color=GT_C, lw=1.0, ls='--', alpha=0.7, label='reference')

    # Estimated trajectory coloured by error
    norm = mcolors.Normalize(vmin=errors.min(), vmax=errors.max())
    cmap = matplotlib.colormaps[ERR_CM]
    for k in range(len(pos_est_aligned) - 1):
        c = cmap(norm((errors[k] + errors[k+1]) / 2))
        ax.plot(pos_est_aligned[k:k+2, 0],
                pos_est_aligned[k:k+2, 1],
                pos_est_aligned[k:k+2, 2],
                color=c, lw=1.5)

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.12, shrink=0.6)
    cbar.set_label('ATE (m)', fontsize=9)

    ax.legend(fontsize=9)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)'); ax.set_zlabel('z (m)')
    t = title + '\n' if title else ''
    ax.set_title(f'{t}ATE w.r.t. translation part (m)\n(with SE(3) alignment)', fontsize=10)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_ate_raw(ts_matched, errors, metrics, out_path, title=''):
    t0   = ts_matched[0]
    t_s  = ts_matched - t0

    rmse   = metrics['ate_rmse']
    mean   = metrics['ate_mean']
    median = metrics['ate_median']
    std    = metrics['ate_std']

    fig, ax = plt.subplots(figsize=(10, 6))
    _style(ax)

    ax.plot(t_s, errors, color=GT_C, lw=1.0, label='ATE (m)')
    ax.axhline(rmse,   color=EST_C,    lw=1.5, label=f'rmse   {rmse:.3f} m')
    ax.axhline(median, color='#55A868', lw=1.5, label=f'median {median:.3f} m')
    ax.axhline(mean,   color='#C44E52', lw=1.5, label=f'mean   {mean:.3f} m')
    ax.axhspan(mean - std, mean + std, alpha=0.25, color=EST_C, label=f'±std   {std:.3f} m')

    ax.set_xlabel('t (s)', fontsize=10)
    ax.set_ylabel('ATE (m)', fontsize=10)
    t = title + '\n' if title else ''
    ax.set_title(f'{t}ATE w.r.t. translation part (m)\n(with SE(3) alignment)', fontsize=11)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=9, loc='upper right')
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_rpe(rpe_metrics, gt_path, est_path, max_diff, out_path, title='', delta=1.0):
    """Recompute per-segment RPE errors and plot over cumulative distance."""
    # Load and associate
    ts_gt,  pos_gt,  _ = _load_tum_full(gt_path)
    ts_est, pos_est, _ = _load_tum_full(est_path)

    def _make_T_batch(pos, quats):
        N = len(pos)
        T = np.tile(np.eye(4), (N, 1, 1))
        T[:, :3, 3] = pos
        T[:, :3, :3] = _quat_to_rot(quats)
        return T

    pairs = _associate(ts_gt, ts_est, max_diff)
    T_gt  = _make_T_batch(pos_gt,  np.column_stack([np.zeros((len(pos_gt), 3)),  np.ones(len(pos_gt))]))[
        [j for j, _ in pairs]]
    T_est = _make_T_batch(pos_est, np.column_stack([np.zeros((len(pos_est), 3)), np.ones(len(pos_est))]))[
        [i for _, i in pairs]]

    # Re-do with real quaternions
    quats_gt_m  = np.array([[0,0,0,1]]*len(pos_gt))[:]   # placeholder; redo properly:
    ts_gt2,  pg2, qg2 = _load_tum_full(gt_path)
    ts_est2, pe2, qe2 = _load_tum_full(est_path)
    T_gt  = _make_T_batch(pg2[[j for j,_ in pairs]], qg2[[j for j,_ in pairs]])
    T_est = _make_T_batch(pe2[[i for _,i in pairs]], qe2[[i for _,i in pairs]])

    dists = _cum_dist(np.array([T[:3, 3] for T in T_gt]))

    seg_dists, trans_errs, rot_errs = [], [], []
    i = 0
    while i < len(T_gt):
        target = dists[i] + delta
        j = int(np.searchsorted(dists, target))
        if j >= len(T_gt):
            break
        Q_gt  = np.linalg.inv(T_gt[i])  @ T_gt[j]
        Q_est = np.linalg.inv(T_est[i]) @ T_est[j]
        E     = np.linalg.inv(Q_gt) @ Q_est
        trans_errs.append(np.linalg.norm(E[:3, 3]))
        cos = np.clip((np.trace(E[:3, :3]) - 1) / 2, -1, 1)
        rot_errs.append(np.degrees(np.arccos(cos)))
        seg_dists.append(dists[i])
        i = j

    seg_dists  = np.array(seg_dists)
    trans_errs = np.array(trans_errs)
    rot_errs   = np.array(rot_errs)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.patch.set_facecolor(BG)

    for ax, errs, ylabel, color in [
        (axes[0], trans_errs, 'trans error (m)',   EST_C),
        (axes[1], rot_errs,   'rot error (deg)',   '#C44E52'),
    ]:
        _style(ax)
        rmse   = float(np.sqrt(np.mean(errs**2)))
        mean_v = float(np.mean(errs))
        std_v  = float(np.std(errs))
        ax.plot(seg_dists, errs, color=GT_C, lw=1.0, label='RPE')
        ax.axhline(rmse,   color=color,      lw=1.5, label=f'rmse   {rmse:.4f}')
        ax.axhline(mean_v, color='#C44E52',  lw=1.2, ls='--', label=f'mean   {mean_v:.4f}')
        ax.axhspan(mean_v - std_v, mean_v + std_v, alpha=0.2, color=color, label=f'±std   {std_v:.4f}')
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=8, loc='upper right')

    axes[1].set_xlabel('distance (m)', fontsize=10)
    t = title + '\n' if title else ''
    axes[0].set_title(f'{t}RPE (delta={delta} m)', fontsize=11)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_xyz(ts_gt, pos_gt, ts_est, pos_est, out_path, title=''):
    t0 = min(ts_gt[0], ts_est[0])

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.patch.set_facecolor(BG)

    for ax, dim, label in zip(axes, range(3), ['x (m)', 'y (m)', 'z (m)']):
        _style(ax)
        ax.plot(ts_gt  - t0, pos_gt[:, dim],  color=GT_C,  lw=1.1, ls='--', label='reference')
        ax.plot(ts_est - t0, pos_est[:, dim], color=EST_C, lw=1.1, ls='-',  label='estimated')
        ax.set_ylabel(label, fontsize=10)
        ax.legend(fontsize=8, loc='upper right')

    axes[2].set_xlabel('t (s)', fontsize=10)
    t = f'{title}\n' if title else ''
    axes[0].set_title(f'{t}x / y / z vs time', fontsize=11)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_speed(ts_est, pos_est, out_path, title=''):
    dt = np.diff(ts_est)
    dt = np.where(dt > 0, dt, np.nan)
    speed = np.linalg.norm(np.diff(pos_est, axis=0), axis=1) / dt
    t_mid = (ts_est[:-1] + ts_est[1:]) / 2

    fig, ax = plt.subplots(figsize=(10, 4))
    _style(ax)
    ax.plot(t_mid, speed, color=EST_C, lw=1.2, label='estimated')
    ax.set_xlabel('t (s)', fontsize=10)
    ax.set_ylabel('v (m/s)', fontsize=10)
    t = f'{title}\n' if title else ''
    ax.set_title(f'{t}Speed', fontsize=11)
    ax.set_ylim(bottom=0)
    ax.legend(fontsize=9)
    plt.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gt',         help='Ground truth TUM file')
    ap.add_argument('est',        help='Estimated trajectory TUM file')
    ap.add_argument('output_dir', help='Directory to write PNGs into')
    ap.add_argument('--ate_json', help='Pre-computed ATE metrics JSON (from ate.py --output)')
    ap.add_argument('--rpe_json', help='Pre-computed RPE metrics JSON (from rpe.py --output)')
    ap.add_argument('--max_diff', type=float, default=0.02,
                    help='Max timestamp diff for association (s, default: 0.02)')
    ap.add_argument('--no_align', action='store_true',
                    help='Skip SE3 alignment')
    ap.add_argument('--delta',    type=float, default=1.0,
                    help='RPE distance step in metres (default: 1.0)')
    ap.add_argument('--title',    default='',
                    help='Optional title prefix for all plots')
    args = ap.parse_args()

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    # ── Load ──────────────────────────────────────────────────────────────────
    ts_gt,  pos_gt,  _     = _load_tum_full(args.gt)
    ts_est, pos_est, quats = _load_tum_full(args.est)

    pairs   = _associate(ts_gt, ts_est, args.max_diff)
    idx_gt  = np.array([j for j, _ in pairs])
    idx_est = np.array([i for _, i in pairs])
    P_gt    = pos_gt[idx_gt]
    P_est   = pos_est[idx_est]

    if args.no_align:
        P_est_aligned = P_est
    else:
        R, t = _align(P_gt, P_est)
        P_est_aligned = (R @ P_est.T).T + t

    errors = np.linalg.norm(P_gt - P_est_aligned, axis=1)
    ts_matched = ts_gt[idx_gt]

    # ── Load or compute metrics for annotations ───────────────────────────────
    if args.ate_json:
        ate_m = json.loads(Path(args.ate_json).read_text())
    else:
        ate_m = {
            'ate_rmse':   float(np.sqrt(np.mean(errors**2))),
            'ate_mean':   float(np.mean(errors)),
            'ate_max':    float(np.max(errors)),
            'ate_std':    float(np.std(errors)),
            'ate_median': float(np.median(errors)),
        }

    rpe_m = json.loads(Path(args.rpe_json).read_text()) if args.rpe_json else {}

    # ── 1. traj.png ──────────────────────────────────────────────────────────
    print('Plotting traj.png ...')
    plot_traj(P_gt, P_est_aligned, out / 'traj.png', title=args.title)

    # ── 2. ate_map.png ───────────────────────────────────────────────────────
    print('Plotting ate_map.png ...')
    plot_ate_map(P_gt, P_est_aligned, errors, out / 'ate_map.png', title=args.title)

    # ── 3. ate_raw.png ───────────────────────────────────────────────────────
    print('Plotting ate_raw.png ...')
    plot_ate_raw(ts_matched, errors, ate_m, out / 'ate_raw.png', title=args.title)

    # ── 4. rpe.png ───────────────────────────────────────────────────────────
    print('Plotting rpe.png ...')
    plot_rpe(rpe_m, args.gt, args.est, args.max_diff,
             out / 'rpe.png', title=args.title, delta=args.delta)

    # ── 5. xyz.png ───────────────────────────────────────────────────────────
    print('Plotting xyz.png ...')
    # Apply the same alignment transform to the full estimated trajectory
    pos_est_full_aligned = pos_est if args.no_align else (R @ pos_est.T).T + t
    plot_xyz(ts_gt, pos_gt, ts_est, pos_est_full_aligned, out / 'xyz.png', title=args.title)

    # ── 6. speed.png ─────────────────────────────────────────────────────────
    print('Plotting speed.png ...')
    plot_speed(ts_est, pos_est, out / 'speed.png', title=args.title)

    print(f'\nAll plots saved to {out}/')
    for p in sorted(out.glob('*.png')):
        print(f'  {p.name}')


if __name__ == '__main__':
    main()

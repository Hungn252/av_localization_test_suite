#!/usr/bin/env python3
"""
kitti_gt_to_tum.py
Extracts KITTI raw ground truth (oxts) and saves as TUM format.

Usage:
    python3 kitti_gt_to_tum.py <kitti_raw_dir> <date> <drive> <output.txt>

Example:
    python3 kitti_gt_to_tum.py ~/suite/bags/raw 2011_10_03 0042 \
        ~/suite/ground_truth/kitti/2011_10_03_drive_0042.txt

Output format (TUM):
    timestamp x y z qx qy qz qw
    (positions in metres, origin = first frame)
"""

import sys
import numpy as np
from pathlib import Path

try:
    import pykitti
except ImportError:
    print("[ERROR] pykitti not installed. Run: pip install pykitti")
    sys.exit(1)

if len(sys.argv) < 5:
    print(__doc__)
    sys.exit(1)

kitti_dir = sys.argv[1]
date      = sys.argv[2]
drive     = sys.argv[3]
out_path  = Path(sys.argv[4])
out_path.parent.mkdir(parents=True, exist_ok=True)

print(f"[INFO]  Loading {date} drive {drive}...")
data = pykitti.raw(kitti_dir, date, drive, dataset='extract')
poses = list(data.oxts)
print(f"[INFO]  Frames: {len(poses)}")


def rot_to_quat(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s


with open(out_path, "w") as f:
    for oxts, timestamp in zip(poses, data.timestamps):
        T  = oxts.T_w_imu
        x, y, z = T[0,3], T[1,3], T[2,3]
        qx, qy, qz, qw = rot_to_quat(T[:3,:3])
        ts = timestamp.timestamp()
        f.write(f"{ts:.9f} {x:.6f} {y:.6f} {z:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

print(f"[INFO]  Saved {len(poses)} poses → {out_path}")

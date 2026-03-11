#!/bin/bash
# kitti_prepare.sh
# Converts a KITTI raw sequence into a ROS2 bag + TUM ground truth.
#
# Usage:
#   ./kitti_prepare.sh <kitti_raw_dir> <date> <drive> <bag_output_path> <gt_output_path>
#
# Example:
#   ./kitti_prepare.sh ~/suite/bags/raw 2011_10_03 0042 \
#       ~/suite/bags/kitti/2011_10_03_drive_0042 \
#       ~/suite/ground_truth/kitti/2011_10_03_drive_0042.txt

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[kitti_prepare]${NC} $*"; }
warn() { echo -e "${YELLOW}[kitti_prepare]${NC} $*"; }
err()  { echo -e "${RED}[kitti_prepare]${NC} $*"; exit 1; }

if [ "$#" -ne 5 ]; then
    echo "Usage: $0 <kitti_raw_dir> <date> <drive> <bag_output_path> <gt_output_path>"
    echo ""
    echo "Example:"
    echo "  $0 ~/suite/bags/raw 2011_10_03 0042 \\"
    echo "     ~/suite/bags/kitti/2011_10_03_drive_0042 \\"
    echo "     ~/suite/ground_truth/kitti/2011_10_03_drive_0042.txt"
    exit 1
fi

KITTI_RAW_DIR="$1"
DATE="$2"
DRIVE="$3"
BAG_OUT="$4"
GT_OUT="$5"
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"

log "KITTI raw dir: $KITTI_RAW_DIR"
log "Date:          $DATE"
log "Drive:         $DRIVE"
log "Bag output:    $BAG_OUT"
log "GT output:     $GT_OUT"
echo ""

# ── Validate ──────────────────────────────────────────────────────────────────
DRIVE_DIR_EXTRACT="$KITTI_RAW_DIR/$DATE/${DATE}_drive_${DRIVE}_extract"
DRIVE_DIR_SYNC="$KITTI_RAW_DIR/$DATE/${DATE}_drive_${DRIVE}_sync"
if [ ! -d "$DRIVE_DIR_EXTRACT" ] && [ ! -d "$DRIVE_DIR_SYNC" ]; then
    err "Drive directory not found. Tried:
  $DRIVE_DIR_EXTRACT
  $DRIVE_DIR_SYNC"
fi

CALIB="$KITTI_RAW_DIR/$DATE/calib_imu_to_velo.txt"
[ ! -f "$CALIB" ] && err "Calibration file not found: $CALIB"

[ -d "$BAG_OUT" ] && err "Bag already exists: $BAG_OUT — delete it first."
[ -f "$GT_OUT"  ] && err "GT file already exists: $GT_OUT — delete it first."

log "Input validated ✓"

# ── Step 1: ROS2 bag ──────────────────────────────────────────────────────────
log "Step 1: Converting to ROS2 bag..."
python3 "$SCRIPTS_DIR/kitti_to_ros2.py" "$KITTI_RAW_DIR" "$DATE" "$DRIVE" "$BAG_OUT"

# ── Step 2: Ground truth ──────────────────────────────────────────────────────
log "Step 2: Extracting ground truth..."
python3 "$SCRIPTS_DIR/kitti_gt_to_tum.py" "$KITTI_RAW_DIR" "$DATE" "$DRIVE" "$GT_OUT"

# ── Step 3: Verify ────────────────────────────────────────────────────────────
log "Step 3: Verifying..."
BAG_FRAMES=$(python3 -c "
from rosbags.rosbag2 import Reader
count = 0
with Reader('$BAG_OUT') as r:
    for conn, ts, data in r.messages():
        if conn.topic == '/kitti/velo/pointcloud':
            count += 1
print(count)
")
GT_FRAMES=$(wc -l < "$GT_OUT")

log "Bag LiDAR frames: $BAG_FRAMES"
log "GT frames:        $GT_FRAMES"

if [ "$BAG_FRAMES" != "$GT_FRAMES" ]; then
    warn "Frame count mismatch: bag=$BAG_FRAMES gt=$GT_FRAMES"
else
    log "Frame counts match ✓"
fi

echo ""
log "=== Done ==="
log "Bag: $BAG_OUT"
log "GT:  $GT_OUT"
echo ""
log "Next step:"
log "  ~/suite/scripts/evaluate.sh $BAG_OUT $GT_OUT /kitti/velo/pointcloud ${DATE}_drive_${DRIVE}"

#!/bin/bash
# run_algorithm.sh
# Runs a SLAM algorithm on a ROS2 bag and outputs poses in TUM format.
#
# Usage:
#   ./run_algorithm.sh <algorithm> <bag_path> <lidar_topic> <output_dir>
#
# Supported algorithms:
#   kiss_icp
#
# Output:
#   <output_dir>/poses_tum.txt
#   <output_dir>/poses_kitti.txt   (kiss_icp)
#   <output_dir>/algorithm_metrics.txt
#
# Example:
#   ./run_algorithm.sh kiss_icp ~/suite/bags/kitti/2011_10_03_drive_0042 \
#       /kitti/velo/pointcloud ~/suite/runs/kitti_0042_kiss

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[run_algorithm]${NC} $*"; }
warn() { echo -e "${YELLOW}[run_algorithm]${NC} $*"; }
err()  { echo -e "${RED}[run_algorithm]${NC} $*"; exit 1; }

if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <algorithm> <bag_path> <lidar_topic> <output_dir>"
    echo ""
    echo "Supported algorithms: kiss_icp"
    echo ""
    echo "Example:"
    echo "  $0 kiss_icp ~/suite/bags/kitti/2011_10_03_drive_0042 \\"
    echo "     /kitti/velo/pointcloud ~/suite/runs/kitti_0042_kiss"
    exit 1
fi

ALGORITHM="$1"
BAG_PATH="$2"
LIDAR_TOPIC="$3"
OUTPUT_DIR="$4"
SUITE_DIR="$HOME/suite"

[ ! -d "$BAG_PATH" ] && [ ! -f "$BAG_PATH" ] && err "Bag not found: $BAG_PATH"
[ -d "$OUTPUT_DIR" ] && err "Output dir already exists: $OUTPUT_DIR — delete it first."

mkdir -p "$OUTPUT_DIR"

log "Algorithm:  $ALGORITHM"
log "Bag:        $BAG_PATH"
log "Topic:      $LIDAR_TOPIC"
log "Output dir: $OUTPUT_DIR"
echo ""

case "$ALGORITHM" in

    kiss_icp)
        KISS_ICP="$SUITE_DIR/venv/kiss_icp/bin/kiss_icp_pipeline"
        [ ! -f "$KISS_ICP" ] && err "KISS-ICP not found at $KISS_ICP
  Recreate venv: python3 -m venv $SUITE_DIR/venv/kiss_icp
                 $SUITE_DIR/venv/kiss_icp/bin/pip install rosbags==0.9.23 kiss-icp==1.2.3"

        RAW_OUT="$OUTPUT_DIR/raw"
        export kiss_icp_out_dir="$RAW_OUT"

        log "Running KISS-ICP..."
        "$KISS_ICP" --topic "$LIDAR_TOPIC" "$BAG_PATH"

        POSES_TUM=$(find "$RAW_OUT" -name "*poses_tum.txt"   | head -1)
        POSES_KIT=$(find "$RAW_OUT" -name "*poses_kitti.txt" | head -1)
        METRICS=$(find   "$RAW_OUT" -name "result_metrics.log" | head -1)

        [ -z "$POSES_TUM" ] && err "KISS-ICP did not produce poses_tum.txt — check output above."

        cp "$POSES_TUM" "$OUTPUT_DIR/poses_tum.txt"
        [ -n "$POSES_KIT" ] && cp "$POSES_KIT" "$OUTPUT_DIR/poses_kitti.txt"
        [ -n "$METRICS"   ] && cp "$METRICS"   "$OUTPUT_DIR/algorithm_metrics.txt"
        ;;

    # ── Add new algorithms here ───────────────────────────────────────────────
    # example_algo)
    #     ...run example_algo...
    #     cp <output_poses> "$OUTPUT_DIR/poses_tum.txt"
    #     ;;

    *)
        err "Unknown algorithm '$ALGORITHM'. Supported: kiss_icp"
        ;;
esac

echo ""
log "=== Done ==="
log "Poses TUM: $OUTPUT_DIR/poses_tum.txt"
echo ""
log "Next step:"
log "  ~/suite/scripts/evaluate.sh $OUTPUT_DIR/poses_tum.txt <gt_path> $ALGORITHM <case_type>"

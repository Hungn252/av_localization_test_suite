#!/bin/bash
# evaluate.sh
# Evaluates a trajectory estimate against ground truth.
# Saves results under ~/suite/results/<algorithm>/<case_type>/<timestamp>/
# and updates ~/suite/results/benchmark.csv and benchmark.md.
#
# Usage:
#   ./evaluate.sh <poses_tum> <gt_path> <algorithm> <case_type> [metrics_file]
#
# Arguments:
#   poses_tum     Path to estimated trajectory in TUM format
#   gt_path       Path to ground truth in TUM format
#   algorithm     Algorithm name (e.g. kiss_icp, loam, fastlio)
#   case_type     Scenario label (e.g. ideal, heavy_rain, night, kitti_0042)
#   metrics_file  Optional: algorithm timing log (algorithm_metrics.txt or per-frame ms file)
#                 Defaults to algorithm_metrics.txt in the same directory as poses_tum
#
# Examples:
#   ./evaluate.sh ~/suite/runs/kitti_kiss/poses_tum.txt \
#                 ~/suite/ground_truth/kitti/2011_10_03_drive_0042.txt \
#                 kiss_icp kitti_0042
#
#   ./evaluate.sh ~/suite/runs/ideal_kiss/poses_tum.txt \
#                 ~/suite/ground_truth/ideal/run_01.txt \
#                 kiss_icp ideal

set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[evaluate]${NC} $*"; }
warn() { echo -e "${YELLOW}[evaluate]${NC} $*"; }
err()  { echo -e "${RED}[evaluate]${NC} $*"; exit 1; }

if [ "$#" -lt 4 ]; then
    echo "Usage: $0 <poses_tum> <gt_path> <algorithm> <case_type> [metrics_file]"
    echo ""
    echo "Examples:"
    echo "  $0 ~/suite/runs/kitti_kiss/poses_tum.txt \\"
    echo "     ~/suite/ground_truth/kitti/2011_10_03_drive_0042.txt \\"
    echo "     kiss_icp kitti_0042"
    echo ""
    echo "  $0 ~/suite/runs/ideal_kiss/poses_tum.txt \\"
    echo "     ~/suite/ground_truth/ideal/run_01.txt \\"
    echo "     kiss_icp ideal"
    exit 1
fi

POSES_TUM="$1"
GT_PATH="$2"
ALGORITHM="$3"
CASE_TYPE="$4"
METRICS_FILE="${5:-$(dirname "$POSES_TUM")/algorithm_metrics.txt}"

SUITE_DIR="$HOME/suite"
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"
METRICS_DIR="$SCRIPTS_DIR/metrics"
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
OUT_DIR="$SUITE_DIR/results/$ALGORITHM/$CASE_TYPE/$TIMESTAMP"

[ ! -f "$POSES_TUM" ] && err "Poses file not found: $POSES_TUM"
[ ! -f "$GT_PATH"   ] && err "Ground truth not found: $GT_PATH"

mkdir -p "$OUT_DIR"
cp "$POSES_TUM" "$OUT_DIR/poses_tum.txt"

log "Algorithm:  $ALGORITHM"
log "Case type:  $CASE_TYPE"
log "Poses:      $POSES_TUM"
log "GT:         $GT_PATH"
log "Output:     $OUT_DIR"
echo ""

# ── Step 1: ATE ───────────────────────────────────────────────────────────────
log "Step 1: Computing ATE..."
python3 "$METRICS_DIR/ate.py" \
    "$GT_PATH" "$OUT_DIR/poses_tum.txt" \
    --output "$OUT_DIR/ate.json" \
    2>&1 | tee "$OUT_DIR/ate.txt"
log "ATE done ✓"
echo ""

# ── Step 2: RPE ───────────────────────────────────────────────────────────────
log "Step 2: Computing RPE..."
python3 "$METRICS_DIR/rpe.py" \
    "$GT_PATH" "$OUT_DIR/poses_tum.txt" \
    --delta 1 \
    --output "$OUT_DIR/rpe.json" \
    2>&1 | tee "$OUT_DIR/rpe.txt"
log "RPE done ✓"
echo ""

# ── Step 3: Latency ───────────────────────────────────────────────────────────
LATENCY_JSON=""
if [ -f "$METRICS_FILE" ]; then
    log "Step 3: Computing latency from $METRICS_FILE..."
    python3 "$METRICS_DIR/latency.py" \
        "$METRICS_FILE" \
        --output "$OUT_DIR/latency.json" \
        2>&1 | tee "$OUT_DIR/latency.txt"
    LATENCY_JSON="$OUT_DIR/latency.json"
    log "Latency done ✓"
else
    warn "Step 3: No metrics file found at $METRICS_FILE — skipping latency"
fi
echo ""

# ── Step 4: Plots ────────────────────────────────────────────────────────────
log "Step 4: Generating plots..."
python3 "$METRICS_DIR/plot.py" \
    "$GT_PATH" "$OUT_DIR/poses_tum.txt" \
    "$OUT_DIR" \
    --ate_json "$OUT_DIR/ate.json" \
    --rpe_json "$OUT_DIR/rpe.json" \
    --title "$ALGORITHM / $CASE_TYPE" \
    2>&1 | grep -E "^Plotting|^All|^\s{2}" || warn "Some plots failed — continuing"
log "Plots done ✓"
echo ""

# ── Step 5: Update benchmark report ──────────────────────────────────────────
log "Step 5: Updating benchmark report..."

python3 - <<PYEOF
import csv, json
from pathlib import Path
from datetime import datetime
from collections import defaultdict

suite_dir   = Path("$SUITE_DIR")
algorithm   = "$ALGORITHM"
case_type   = "$CASE_TYPE"
timestamp   = "$TIMESTAMP"
out_dir     = "$OUT_DIR"
latency_json = "$LATENCY_JSON"

ate_path = Path(out_dir) / "ate.json"
rpe_path = Path(out_dir) / "rpe.json"

ate = json.loads(ate_path.read_text())
rpe = json.loads(rpe_path.read_text())
lat = json.loads(Path(latency_json).read_text()) if latency_json and Path(latency_json).exists() else {}

csv_path = suite_dir / "results" / "benchmark.csv"
md_path  = suite_dir / "results" / "benchmark.md"

FIELDS = [
    "algorithm", "case_type",
    "ate_rmse", "ate_mean", "ate_max", "ate_std", "ate_median",
    "rpe_trans_rmse", "rpe_trans_mean", "rpe_rot_rmse_deg",
    "latency_mean_ms",
    "timestamp", "results_dir",
]

rows = []
if csv_path.exists():
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

new_row = {
    "algorithm":       algorithm,
    "case_type":       case_type,
    "ate_rmse":        f"{ate['ate_rmse']:.4f}",
    "ate_mean":        f"{ate['ate_mean']:.4f}",
    "ate_max":         f"{ate['ate_max']:.4f}",
    "ate_std":         f"{ate['ate_std']:.4f}",
    "ate_median":      f"{ate['ate_median']:.4f}",
    "rpe_trans_rmse":  f"{rpe['trans_rmse']:.4f}",
    "rpe_trans_mean":  f"{rpe['trans_mean']:.4f}",
    "rpe_rot_rmse_deg": f"{rpe['rot_rmse_deg']:.4f}",
    "latency_mean_ms": f"{lat.get('mean_ms', ''):.1f}" if lat.get('mean_ms') is not None else "",
    "timestamp":       timestamp,
    "results_dir":     out_dir,
}

found = False
for i, row in enumerate(rows):
    if row["algorithm"] == algorithm and row["case_type"] == case_type:
        rows[i] = new_row
        found = True
        break
if not found:
    rows.append(new_row)

rows.sort(key=lambda r: (r["algorithm"], r["case_type"]))

csv_path.parent.mkdir(parents=True, exist_ok=True)
with open(csv_path, "w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerows(rows)

def f(v, decimals=3):
    try:    return f"{float(v):.{decimals}f}"
    except: return v or "N/A"

by_algo = defaultdict(list)
for row in rows:
    by_algo[row["algorithm"]].append(row)

lines = [
    "# SLAM Benchmark Report",
    "",
    f"_Last updated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}_",
    "",
    "> Results saved under \`~/suite/results/<algorithm>/<case_type>/<timestamp>/\`",
    "",
]

for algo in sorted(by_algo):
    lines += [f"## {algo}", ""]
    lines += [
        "| Case | ATE RMSE | ATE Mean | ATE Max | RPE Trans RMSE | RPE Rot RMSE | Latency | Last Run |",
        "|------|:--------:|:--------:|:-------:|:--------------:|:------------:|:-------:|----------|",
    ]
    for row in by_algo[algo]:
        lat_str = f"{f(row['latency_mean_ms'])} ms" if row.get('latency_mean_ms') else "N/A"
        lines.append(
            f"| {row['case_type']} "
            f"| {f(row['ate_rmse'])} m | {f(row['ate_mean'])} m | {f(row['ate_max'])} m "
            f"| {f(row['rpe_trans_rmse'])} m | {f(row['rpe_rot_rmse_deg'])} deg "
            f"| {lat_str} | {row['timestamp']} |"
        )
    lines.append("")

md_path.write_text("\n".join(lines))
print(f"Benchmark updated → {md_path}")
PYEOF

# ── Summary ───────────────────────────────────────────────────────────────────
ATE_RMSE=$(python3 -c "import json; d=json.load(open('$OUT_DIR/ate.json')); print(f\"{d['ate_rmse']:.4f}\")")
RPE_RMSE=$(python3 -c "import json; d=json.load(open('$OUT_DIR/rpe.json')); print(f\"{d['trans_rmse']:.4f}\")")
LAT_STR=""
[ -f "$OUT_DIR/latency.json" ] && LAT_STR=$(python3 -c "import json; d=json.load(open('$OUT_DIR/latency.json')); print(f\"{d.get('mean_ms','N/A')} ms\")")

echo ""
log "=== Summary: $ALGORITHM / $CASE_TYPE ==="
log "Output:       $OUT_DIR"
log "ATE RMSE:     ${ATE_RMSE} m"
log "RPE trans RMSE: ${RPE_RMSE} m"
[ -n "$LAT_STR" ] && log "Latency mean: $LAT_STR"
log "Benchmark:    $SUITE_DIR/results/benchmark.md"

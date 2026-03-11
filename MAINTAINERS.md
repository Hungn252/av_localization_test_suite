# Maintainers Guide

Internal reference for suite development, dataset preparation, baseline evaluation,
and Docker image releases.

---

## Directory Structure

```
~/suite/
├── bags/                   # ROS2 bags
│   ├── ideal/
│   │   └── kitti/
│   │       └── run_01/     # KITTI 2011_10_03 drive 0042 (ideal conditions)
│   └── raw/                # KITTI raw data (input for kitti_prepare.sh)
│       └── 2011_10_03/
│           ├── calib_*.txt
│           └── 2011_10_03_drive_0042_extract/
├── baselines/              # Pre-evaluated algorithm metrics (used in benchmark)
│   └── kiss_icp/
│       └── ideal/
│           └── metrics.json
├── configs/
│   └── scenarios.yaml      # Scenario registry (bag, ground truth, topics)
├── ground_truth/           # TUM-format ground truth files
│   └── kitti/
│       └── 2011_10_03_drive_0042.txt
├── results/                # Evaluation outputs
├── runs/                   # Intermediate algorithm pose outputs
├── scripts/
│   ├── metrics/            # ATE, RPE, latency, plot scripts
│   │   ├── ate.py
│   │   ├── rpe.py
│   │   ├── latency.py
│   │   ├── plot.py
│   │   └── benchmark_report.py
│   ├── run_suite.py        # Main orchestrator
│   ├── record_poses.py     # ROS2 pose subscriber/recorder
│   ├── run_algorithm.sh    # Run a known algorithm on a bag
│   ├── evaluate.sh         # Evaluate poses against ground truth
│   └── kitti_prepare.sh    # Convert KITTI raw → ROS2 bag + TUM GT
├── user_config.yaml        # Template shipped to users
├── run_suite.sh            # Entry point
├── Dockerfile
├── build.sh                # Build + push Docker image
└── venv/
    └── kiss_icp/           # rosbags 0.9.23 + kiss-icp 1.2.3
```

---

## Preparing a Dataset

### KITTI raw → ROS2 bag + ground truth

Place the raw sequence under `bags/raw/`:

```
bags/raw/<date>/
├── calib_cam_to_cam.txt
├── calib_imu_to_velo.txt
├── calib_velo_to_cam.txt
└── <date>_drive_<drive>_extract/
    ├── oxts/
    └── velodyne_points/
```

```bash
scripts/kitti_prepare.sh bags/raw 2011_10_03 0042 \
    bags/ideal/kitti/run_01 \
    ground_truth/kitti/2011_10_03_drive_0042.txt
```

---

## Running a Baseline Algorithm

```bash
# Usage: run_algorithm.sh <algorithm> <bag_path> <lidar_topic> <output_dir>
scripts/run_algorithm.sh kiss_icp \
    bags/ideal/kitti/run_01 \
    /kitti/velo/pointcloud \
    runs/ideal_kiss
```

Supported: `kiss_icp`. Output: `<output_dir>/poses_tum.txt` + `algorithm_metrics.txt`.

To add a new algorithm, add a case block to `scripts/run_algorithm.sh`.

---

## Evaluating Poses

```bash
# Usage: evaluate.sh <poses_tum> <gt_path> <algorithm> <case_type>
scripts/evaluate.sh \
    runs/ideal_kiss/poses_tum.txt \
    ground_truth/kitti/2011_10_03_drive_0042.txt \
    kiss_icp ideal
```

Saves to `results/<algorithm>/<case_type>/<timestamp>/` with full metrics JSON,
plots, and updates `results/benchmark.md` + `results/benchmark.csv`.

### Metrics scripts (standalone)

```bash
python3 scripts/metrics/ate.py     <gt_tum> <est_tum> [--output ate.json]
python3 scripts/metrics/rpe.py     <gt_tum> <est_tum> [--delta 1.0] [--output rpe.json]
python3 scripts/metrics/latency.py <metrics_file>     [--output latency.json]
python3 scripts/metrics/plot.py    <gt_tum> <est_tum> <output_dir> [--title "label"]
```

---

## Adding a New Baseline

1. Run the algorithm and evaluate as above.
2. Store the metrics under `baselines/`:

```bash
mkdir -p baselines/my_algo/ideal
python3 - <<'EOF'
import json
from pathlib import Path

ate = json.loads(Path("results/my_algo/ideal/<timestamp>/ate.json").read_text())
rpe = json.loads(Path("results/my_algo/ideal/<timestamp>/rpe.json").read_text())
lat = json.loads(Path("results/my_algo/ideal/<timestamp>/latency.json").read_text())

metrics = {
    "algorithm":        "my_algo",
    "scenario":         "ideal",
    "ate_rmse":         ate["ate_rmse"],
    "ate_mean":         ate["ate_mean"],
    "ate_max":          ate["ate_max"],
    "rpe_trans_rmse":   rpe["trans_rmse"],
    "rpe_trans_mean":   rpe["trans_mean"],
    "rpe_rot_rmse_deg": rpe["rot_rmse_deg"],
    "latency_mean_ms":  lat.get("mean_ms"),
}
Path("baselines/my_algo/ideal/metrics.json").write_text(json.dumps(metrics, indent=2))
EOF
```

The new baseline appears automatically in all future user benchmark reports.

---

## Adding a New Scenario

1. Add the bag to `bags/<scenario>/run_01/`
2. Add ground truth to `ground_truth/<scenario>/run_01.txt`
3. Register it in `configs/scenarios.yaml`:

```yaml
scenarios:
  my_scenario:
    bag: bags/my_scenario/run_01
    ground_truth: ground_truth/my_scenario/run_01.txt
    lidar_topic: /my/lidar/topic
    imu_topic: /my/imu/topic
    duration_s: 120
    description: "Brief description"
```

4. Run all baseline algorithms on the new scenario and store their `metrics.json`.
5. Rebuild and push the Docker image.

---

## GitHub

Code lives in git. Bags are distributed separately and mounted as a volume at runtime.

```bash
git init
git remote add origin https://github.com/<org>/slam-suite.git
git add .
git commit -m "Initial commit"
git push -u origin main
```

`.gitignore` excludes `bags/`, `venv/`, `results/`, `runs/`, `logs/`.

---

## Docker

### Build and push

Bags are not baked into the image — they are mounted at runtime by the user.

```bash
./build.sh ghcr.io/<org>          # GitHub Container Registry
./build.sh <dockerhub-user>       # Docker Hub
./build.sh ghcr.io/<org> v1.0.0  # versioned tag
```

**What goes in the image**: scripts, configs, baselines, ground truth, ROS2, kiss_icp venv
**What is excluded** (`.dockerignore`): `bags/`, `venv/`, `results/`, `carla_run/`

Final image size: ~1.5 GB.

### Layer order (Dockerfile)

```
1. System packages + Python deps   (changes rarely)
2. scripts/, configs/, baselines/  (changes often)
3. kiss_icp venv                   (changes rarely)
```

---

## Dependencies

| Tool | Version | Used for |
|---|---|---|
| ROS2 Humble | — | Bag playback, pose recording |
| pykitti | latest | KITTI raw loading |
| rosbags | 0.11 (system) | KITTI bag writing |
| kiss-icp | 1.2.3 (venv) | Baseline LiDAR odometry |
| rosbags | 0.9.23 (venv) | KISS-ICP bag reading |
| numpy | — | Metric computation |
| matplotlib | — | Plots |
| PyYAML | — | Config parsing |

Two versions of `rosbags` are needed because kiss-icp 1.2.3 requires the older API.
Recreate the venv:

```bash
python3 -m venv venv/kiss_icp
venv/kiss_icp/bin/pip install rosbags==0.9.23 kiss-icp==1.2.3
```

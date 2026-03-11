# Suite — SLAM Evaluation Pipeline

Benchmark your SLAM algorithm against pre-evaluated baselines across multiple sensor scenarios.
Pull the Docker image, configure your algorithm, and run one command.

---

## Quick Start

### 1. Configure your algorithm

Edit `user_config.yaml`:

```yaml
algorithm:
  name: my_slam                    # label shown in benchmark
  pose_topic: /slam/odometry       # topic your algorithm publishes poses on
  pose_type: odometry              # odometry | pose_stamped | pose_with_covariance
  launch_command: "ros2 launch my_slam slam.launch.py"  # or "" to start manually
  launch_wait_s: 5

output_path: /results              # AUT/ folder is written here
```

**Supported pose message types:**

| `pose_type` | ROS2 message |
|---|---|
| `odometry` | `nav_msgs/msg/Odometry` |
| `pose_stamped` | `geometry_msgs/msg/PoseStamped` |
| `pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` |

**Topics published by the suite during evaluation** (your algorithm subscribes to these):

| Scenario | LiDAR | IMU | GPS |
|---|---|---|---|
| `ideal` | `/kitti/velo/pointcloud` | `/kitti/oxts/imu` | `/kitti/oxts/gps` |

If `launch_command` is empty the suite pauses and prints a prompt for you to start
your algorithm manually before pressing Enter.

### 2. Run

```bash
# 1. Pull the image
docker pull ghcr.io/hungn252/slam-suite:latest

# 2. Download the bags (link provided separately) 

# 3. Run
docker run --rm \
  --network host \
  -v $(pwd)/my_config.yaml:/root/suite/user_config.yaml:ro \
  -v /path/to/bags:/root/suite/bags:ro \
  -v $(pwd)/results:/results \
  ghcr.io/<org>/slam-suite:latest
```

`--network host` lets ROS2 DDS reach your algorithm whether it runs on the host
or in a separate container on the same machine.


### 3. Results

Everything is written to `<output_path>/AUT/`:

```
<output_path>/AUT/
├── ideal/
│   ├── poses_tum.txt       # recorded trajectory (TUM format)
│   ├── ate.json            # Absolute Trajectory Error
│   ├── rpe.json            # Relative Pose Error
│   ├── latency.json        # pose publishing latency
│   ├── ate_map.png         # 3D trajectory coloured by ATE error
│   ├── ate_raw.png         # ATE over time with RMSE / mean / ±std
│   ├── rpe.png             # RPE per segment over distance
│   ├── traj.png            # 3D trajectory overlay (GT vs estimated)
│   ├── xyz.png             # x / y / z vs time
│   └── speed.png           # estimated speed vs time
└── benchmark.md            # your algorithm vs all baselines
```

### 4. Benchmark report

`AUT/benchmark.md` compares your algorithm against all pre-evaluated baselines:

```
## Scenario: ideal

| Algorithm     | ATE RMSE | ATE Mean | RPE Trans RMSE | RPE Rot RMSE | Latency  |
|---------------|:--------:|:--------:|:--------------:|:------------:|:--------:|
| kiss_icp      | 20.596 m | 18.496 m |     0.112 m    |  0.059 deg   |  12.0 ms |
| **my_slam**   | ...      | ...      |     ...        |  ...         |  ...     |
```

---

## Scenarios

| Scenario | Description | Status |
|---|---|---|
| `ideal` | KITTI 2011_10_03 drive 0042 | Available |
| `rain` | Heavy rain | Coming soon |
| `night` | Night lighting | Coming soon |
| `high_speed` | High vehicle speed | Coming soon |
| `gps_denial` | GPS signal degraded | Coming soon |
| `rain_night` | Rain + night | Coming soon |

---

## Debugging

Open an interactive shell inside the container:

```bash
docker run --rm -it --network host \
  -v $(pwd)/my_config.yaml:/root/suite/user_config.yaml:ro \
  -v /path/to/bags:/root/suite/bags:ro \
  -v $(pwd)/results:/results \
  --entrypoint bash \
  ghcr.io/<org>/slam-suite:latest
```

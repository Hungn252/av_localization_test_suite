FROM ros:humble-ros-base

# ── System dependencies ───────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
        python3-yaml \
        python3-numpy \
        python3-matplotlib \
        ros-humble-rclpy \
        ros-humble-nav-msgs \
        ros-humble-geometry-msgs \
        ros-humble-sensor-msgs \
        ros-humble-ros2bag \
        ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

# ── Suite root ────────────────────────────────────────────────────────────────
# Placed at /root/suite so that $HOME/suite works naturally inside the container.
WORKDIR /root/suite

# ── Code layers first — cheap to rebuild when bags haven't changed ────────────
COPY scripts/       scripts/
COPY configs/       configs/
COPY baselines/     baselines/
COPY ground_truth/  ground_truth/
COPY run_suite.sh   run_suite.sh
COPY user_config.yaml user_config.yaml

RUN chmod +x run_suite.sh \
             scripts/run_suite.py \
             scripts/record_poses.py \
             scripts/run_algorithm.sh \
             scripts/evaluate.sh \
             scripts/kitti_prepare.sh

# ── KISS-ICP venv (baseline algorithm) ───────────────────────────────────────
RUN python3 -m venv venv/kiss_icp && \
    venv/kiss_icp/bin/pip install --no-cache-dir \
        rosbags==0.9.23 \
        kiss-icp==1.2.3

# ── Source ROS2 for interactive shells ────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ── Runtime interface ─────────────────────────────────────────────────────────
# Mount points expected at runtime:
#   /root/suite/user_config.yaml  — user algorithm config  (read-only)
#   /results                      — output directory        (read-write)
VOLUME ["/results"]
VOLUME ["/root/suite/bags"]

ENTRYPOINT ["/root/suite/run_suite.sh"]

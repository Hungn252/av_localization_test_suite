#!/usr/bin/env python3
"""
record_poses.py — record a ROS2 algorithm pose topic to TUM format.

Usage:
    python3 record_poses.py <output_tum> <topic> <pose_type> <timeout_s>

Arguments:
    output_tum   Path to write TUM-format poses
                 (timestamp tx ty tz qx qy qz qw, one pose per line)
    topic        ROS2 topic name to subscribe to
    pose_type    Message type: odometry | pose_stamped | pose_with_covariance
    timeout_s    Stop recording after this many seconds (also stops on SIGTERM)

On shutdown writes a companion <stem>.latency.json with:
    n_poses, mean_hz, mean_latency_ms, p95_latency_ms
    (latency = wall-clock arrival time minus header.stamp; meaningful only
     when the bag plays in real-time mode)
"""

import json
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


# ── helpers ──────────────────────────────────────────────────────────────────

def _stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _pose_from_odometry(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    stamp = msg.header.stamp
    return stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w


def _pose_from_pose_stamped(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    stamp = msg.header.stamp
    return stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w


def _pose_from_pose_with_covariance(msg):
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    stamp = msg.header.stamp
    return stamp, p.x, p.y, p.z, o.x, o.y, o.z, o.w


# ── node ─────────────────────────────────────────────────────────────────────

class PoseRecorder(Node):
    def __init__(self, topic, pose_type, output_tum, timeout_s):
        super().__init__('pose_recorder')

        self._output_tum  = Path(output_tum)
        self._timeout_s   = float(timeout_s)
        self._start_wall  = time.time()
        self._last_msg_wall = None

        self._poses       = []   # list of (ts_sec, tx, ty, tz, qx, qy, qz, qw)
        self._arrivals    = []   # (wall_clock_s, header_stamp_s) per message

        # Select message type
        pose_type = pose_type.lower()
        if pose_type == 'odometry':
            msg_class   = Odometry
            extractor   = _pose_from_odometry
        elif pose_type == 'pose_stamped':
            msg_class   = PoseStamped
            extractor   = _pose_from_pose_stamped
        elif pose_type == 'pose_with_covariance':
            msg_class   = PoseWithCovarianceStamped
            extractor   = _pose_from_pose_with_covariance
        else:
            self.get_logger().error(
                f'Unknown pose_type "{pose_type}". '
                'Use: odometry | pose_stamped | pose_with_covariance'
            )
            raise ValueError(pose_type)

        self._extractor = extractor
        self._sub = self.create_subscription(
            msg_class, topic, self._callback, 100
        )

        # Timer: check timeout and warn if no messages yet
        self._warn_fired = False
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Recording {pose_type} on {topic} → {self._output_tum}'
        )
        self.get_logger().info(f'Timeout: {timeout_s}s')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _callback(self, msg):
        wall = time.time()
        stamp, tx, ty, tz, qx, qy, qz, qw = self._extractor(msg)
        ts_sec = _stamp_to_sec(stamp)
        self._poses.append((ts_sec, tx, ty, tz, qx, qy, qz, qw))
        self._arrivals.append((wall, ts_sec))
        self._last_msg_wall = wall

    def _tick(self):
        elapsed = time.time() - self._start_wall

        # Warn once if no message in first 10 s
        if (not self._warn_fired
                and elapsed >= 10.0
                and self._last_msg_wall is None):
            self.get_logger().warn(
                'No messages received in the first 10 s — '
                'check that the algorithm is running and the topic/pose_type are correct.'
            )
            self._warn_fired = True

        # Stop when timeout reached
        if elapsed >= self._timeout_s:
            self.get_logger().info('Timeout reached — shutting down.')
            rclpy.shutdown()

    # ── shutdown ──────────────────────────────────────────────────────────────

    def flush(self):
        """Write TUM file and latency companion JSON."""
        n = len(self._poses)
        print(f'[record_poses] Recorded {n} poses.', flush=True)

        self._output_tum.parent.mkdir(parents=True, exist_ok=True)
        with open(self._output_tum, 'w') as f:
            for ts, tx, ty, tz, qx, qy, qz, qw in self._poses:
                f.write(f'{ts:.9f} {tx:.9f} {ty:.9f} {tz:.9f} '
                        f'{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n')

        # Latency companion
        stem = self._output_tum.with_suffix('')
        lat_path = Path(str(stem) + '.latency.json')

        if n >= 2 and self._arrivals:
            arrivals = np.array(self._arrivals)  # (N, 2): [wall, header]
            latencies_ms = (arrivals[:, 0] - arrivals[:, 1]) * 1000.0

            wall_times = arrivals[:, 0]
            duration_s = wall_times[-1] - wall_times[0]
            mean_hz = (n - 1) / duration_s if duration_s > 0 else float('nan')

            lat_data = {
                'n_poses':          n,
                'mean_hz':          float(mean_hz),
                'mean_latency_ms':  float(np.mean(latencies_ms)),
                'p95_latency_ms':   float(np.percentile(latencies_ms, 95)),
            }
        else:
            lat_data = {
                'n_poses':          n,
                'mean_hz':          0.0,
                'mean_latency_ms':  None,
                'p95_latency_ms':   None,
            }

        with open(lat_path, 'w') as f:
            json.dump(lat_data, f, indent=2)
        print(f'[record_poses] Latency data → {lat_path}', flush=True)


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) != 5:
        print(
            'Usage: python3 record_poses.py <output_tum> <topic> <pose_type> <timeout_s>\n'
            '  pose_type: odometry | pose_stamped | pose_with_covariance',
            file=sys.stderr,
        )
        sys.exit(1)

    output_tum = sys.argv[1]
    topic      = sys.argv[2]
    pose_type  = sys.argv[3]
    timeout_s  = sys.argv[4]

    rclpy.init()
    recorder = PoseRecorder(topic, pose_type, output_tum, timeout_s)

    # Handle SIGTERM gracefully
    def _sigterm_handler(signum, frame):
        print('[record_poses] SIGTERM received — flushing and exiting.', flush=True)
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        rclpy.spin(recorder)
    except Exception:
        pass
    finally:
        recorder.flush()
        recorder.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

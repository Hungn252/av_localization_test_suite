#!/usr/bin/env python3
"""
kitti_to_ros2.py
Converts KITTI raw sequence (LiDAR + IMU/GPS only) to ROS2 bag.

Usage:
    python3 kitti_to_ros2.py <kitti_raw_dir> <date> <drive> <output_bag>

Example:
    python3 kitti_to_ros2.py ~/suite/bags/raw 2011_10_03 0042 \
        ~/suite/bags/kitti/2011_10_03_drive_0042

Output topics:
    /kitti/velo/pointcloud  → sensor_msgs/PointCloud2
    /kitti/oxts/imu         → sensor_msgs/Imu
    /kitti/oxts/gps         → sensor_msgs/NavSatFix
"""

import sys
import argparse
import numpy as np
from pathlib import Path

try:
    import pykitti
except ImportError:
    print("[ERROR] pykitti not installed. Run: pip install pykitti")
    sys.exit(1)

try:
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_typestore
except ImportError:
    print("[ERROR] rosbags not installed. Run: pip install rosbags")
    sys.exit(1)


def log(msg): print(f"[INFO]  {msg}")
def err(msg): print(f"[ERROR] {msg}", file=sys.stderr); sys.exit(1)


def make_header(typestore, stamp_ns, frame_id):
    Header = typestore.types["std_msgs/msg/Header"]
    Time   = typestore.types["builtin_interfaces/msg/Time"]
    return Header(
        stamp=Time(sec=int(stamp_ns // 1_000_000_000),
                   nanosec=int(stamp_ns % 1_000_000_000)),
        frame_id=frame_id)


def make_pointcloud2(typestore, stamp_ns, points):
    PC2  = typestore.types["sensor_msgs/msg/PointCloud2"]
    PF   = typestore.types["sensor_msgs/msg/PointField"]
    pts  = points[:, :4].astype(np.float32)
    fields = [
        PF(name="x",         offset=0,  datatype=7, count=1),
        PF(name="y",         offset=4,  datatype=7, count=1),
        PF(name="z",         offset=8,  datatype=7, count=1),
        PF(name="intensity", offset=12, datatype=7, count=1),
    ]
    return PC2(header=make_header(typestore, stamp_ns, "velo_link"),
               height=1, width=len(pts), fields=fields,
               is_bigendian=False, point_step=16, row_step=16*len(pts),
               data=np.frombuffer(pts.tobytes(), dtype=np.uint8), is_dense=True)


def make_imu(typestore, stamp_ns, p):
    Imu  = typestore.types["sensor_msgs/msg/Imu"]
    Vec3 = typestore.types["geometry_msgs/msg/Vector3"]
    Quat = typestore.types["geometry_msgs/msg/Quaternion"]
    cy, sy = np.cos(p.yaw/2),   np.sin(p.yaw/2)
    cp, sp = np.cos(p.pitch/2), np.sin(p.pitch/2)
    cr, sr = np.cos(p.roll/2),  np.sin(p.roll/2)
    return Imu(
        header=make_header(typestore, stamp_ns, "imu_link"),
        orientation=Quat(x=sr*cp*cy - cr*sp*sy,
                         y=cr*sp*cy + sr*cp*sy,
                         z=cr*cp*sy - sr*sp*cy,
                         w=cr*cp*cy + sr*sp*sy),
        orientation_covariance=np.zeros(9, dtype=np.float64),
        angular_velocity=Vec3(x=float(p.wx), y=float(p.wy), z=float(p.wz)),
        angular_velocity_covariance=np.zeros(9, dtype=np.float64),
        linear_acceleration=Vec3(x=float(p.ax), y=float(p.ay), z=float(p.az)),
        linear_acceleration_covariance=np.zeros(9, dtype=np.float64))


def make_navsatfix(typestore, stamp_ns, p):
    NSF = typestore.types["sensor_msgs/msg/NavSatFix"]
    NSS = typestore.types["sensor_msgs/msg/NavSatStatus"]
    return NSF(header=make_header(typestore, stamp_ns, "imu_link"),
               status=NSS(status=0, service=1),
               latitude=float(p.lat), longitude=float(p.lon), altitude=float(p.alt),
               position_covariance=np.zeros(9, dtype=np.float64),
               position_covariance_type=0)


def main():
    parser = argparse.ArgumentParser(description="KITTI raw → ROS2 bag")
    parser.add_argument("kitti_raw_dir")
    parser.add_argument("date")
    parser.add_argument("drive")
    parser.add_argument("output_bag")
    args = parser.parse_args()

    output_bag = Path(args.output_bag)
    if output_bag.exists():
        err(f"Output already exists: {output_bag} — delete it first.")

    log(f"Loading KITTI {args.date} drive {args.drive}...")
    try:
        data = pykitti.raw(args.kitti_raw_dir, args.date, args.drive, dataset='extract')
    except Exception as e:
        err(f"Failed to load KITTI data: {e}")

    total = len(data.velo_files)
    log(f"LiDAR frames: {total} | OXTS frames: {len(data.oxts_files)}")
    if total == 0:
        err("No LiDAR files found.")

    # Load LiDAR-specific timestamps (velodyne_points/timestamps.txt)
    # Use naive datetimes (no timezone) to match pykitti's OXTS timestamp parsing
    from datetime import datetime
    velo_ts_file = Path(args.kitti_raw_dir) / args.date / f"{args.date}_drive_{args.drive}_extract" / "velodyne_points" / "timestamps.txt"
    velo_timestamps = []
    with open(velo_ts_file) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # Timestamps have nanosecond precision (9 decimal digits); truncate to microseconds for strptime
            if '.' in line:
                base, frac = line.rsplit('.', 1)
                line = f"{base}.{frac[:6]}"
            dt = datetime.strptime(line, "%Y-%m-%d %H:%M:%S.%f")
            velo_timestamps.append(dt)
    log(f"LiDAR timestamps loaded: {len(velo_timestamps)}")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    log(f"Writing → {output_bag}")

    with Writer(output_bag, version=8) as writer:
        c_lidar = writer.add_connection("/kitti/velo/pointcloud", "sensor_msgs/msg/PointCloud2", typestore=typestore)
        c_imu   = writer.add_connection("/kitti/oxts/imu",        "sensor_msgs/msg/Imu",         typestore=typestore)
        c_gps   = writer.add_connection("/kitti/oxts/gps",        "sensor_msgs/msg/NavSatFix",   typestore=typestore)

        log("Writing LiDAR...")
        for i, (ts, velo) in enumerate(zip(velo_timestamps, data.velo)):
            stamp_ns = int(ts.timestamp() * 1_000_000_000)
            msg = make_pointcloud2(typestore, stamp_ns, velo[:, :4])
            writer.write(c_lidar, stamp_ns, typestore.serialize_cdr(msg, "sensor_msgs/msg/PointCloud2"))
            if i % 100 == 0:
                log(f"  {i}/{total}")

        log("Writing IMU + GPS...")
        for i, oxts in enumerate(data.oxts):
            ts       = data.timestamps[min(i, len(data.timestamps)-1)]
            stamp_ns = int(ts.timestamp() * 1_000_000_000)
            p        = oxts.packet
            writer.write(c_imu, stamp_ns, typestore.serialize_cdr(make_imu(typestore, stamp_ns, p),       "sensor_msgs/msg/Imu"))
            writer.write(c_gps, stamp_ns, typestore.serialize_cdr(make_navsatfix(typestore, stamp_ns, p), "sensor_msgs/msg/NavSatFix"))

    log(f"Done ✓  {total} frames → {output_bag}")


if __name__ == "__main__":
    main()

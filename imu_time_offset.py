#!/usr/bin/env python3
"""Estimate time offset between IMU gyro and an odometry/pose source by
cross-correlating angular-rate magnitudes. A wrong offset is a top cause of bad LIO/VIO.
Usage: imu_time_offset.py <bag_dir> <imu_topic> <odom_topic> [max_offset_s=0.3]
Set MODE=yaw for signed gyro-z vs yaw-rate (better for legged/ground robots). Positive result = odom stamps lag the IMU (shift odom earlier by that amount)."""
import os
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    for t in topics:
        if t not in types:
            sys.exit(f"{t} not in bag; available: {list(types)}")
    cls = {t: get_message(types[t]) for t in topics}
    out = {t: [] for t in topics}
    while r.has_next():
        n, d, _ = r.read_next()
        if n in out:
            out[n].append(deserialize_message(d, cls[n]))
    return out


def yaw_rate(msgs):
    """Signed yaw rate (rad/s) from odom orientation: robust for ground robots whose gyro norm is dominated by gait/vibration."""
    t, y = [], []
    for m in msgs:
        p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
        o = p.orientation
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        y.append(np.arctan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)))
    t, y = np.array(t), np.unwrap(np.array(y))
    return t[1:], np.diff(y) / np.diff(t)


def quat_rate(msgs):
    t, q = [], []
    for m in msgs:
        p = m.pose.pose if hasattr(m.pose, "pose") else m.pose
        o = p.orientation
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        q.append([o.x, o.y, o.z, o.w])
    t, q = np.array(t), np.array(q)
    dots = np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)
    ang = 2 * np.arccos(dots)
    return t[1:], ang / np.diff(t)


def main(bag, imu_t, odom_t, max_off=0.3):
    d = read(bag, [imu_t, odom_t])
    ti = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in d[imu_t]])
    if os.environ.get("MODE", "norm") == "yaw":   # MODE=yaw: signed gyro-z vs odom yaw rate
        wi = np.array([m.angular_velocity.z for m in d[imu_t]])
        to, wo = yaw_rate(d[odom_t])
    else:
        wi = np.array([np.linalg.norm([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]) for m in d[imu_t]])
        to, wo = quat_rate(d[odom_t])
    dt = 0.005
    grid = np.arange(max(ti[0], to[0]) + 1, min(ti[-1], to[-1]) - 1, dt)
    a = np.interp(grid, ti, wi); a -= a.mean()
    best, best_off = -2, 0.0
    for k in range(-int(max_off / dt), int(max_off / dt) + 1):
        b = np.interp(grid + k * dt, to, wo); b -= b.mean()
        c = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12)
        if c > best:
            best, best_off = c, k * dt
    print(f"estimated offset: {best_off:+.3f} s (correlation {best:.3f})")


if __name__ == "__main__":
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], sys.argv[3], *(float(x) for x in sys.argv[4:5]))

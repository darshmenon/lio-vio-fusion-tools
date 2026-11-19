#!/usr/bin/env python3
"""Copy a ROS2 bag while injecting sensor faults, to test LIO/VIO/EKF robustness offline (no sim).
Usage (ROS2 sourced): inject_faults.py IN_BAG OUT_BAG [options]
  --imu-topic /imu --lidar-topic /points
  --drop-imu F          drop a fraction F (0-1) of IMU messages at random
  --imu-shift S         add S seconds to IMU header stamps (clock offset)
  --imu-noise SIGMA     add gaussian noise (rad/s, m/s^2) to IMU
  --blank-lidar A,B     drop lidar messages between A and B seconds after bag start
All other topics are copied untouched. Seeded for repeatability."""
import argparse
import random
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("inb"); ap.add_argument("outb")
ap.add_argument("--imu-topic", default="/imu"); ap.add_argument("--lidar-topic", default="/points")
ap.add_argument("--drop-imu", type=float, default=0.0); ap.add_argument("--imu-shift", type=float, default=0.0)
ap.add_argument("--imu-noise", type=float, default=0.0); ap.add_argument("--blank-lidar", default="")
ap.add_argument("--seed", type=int, default=0)
a = ap.parse_args()
rnd = random.Random(a.seed)
blank = tuple(float(x) for x in a.blank_lidar.split(",")) if a.blank_lidar else None

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=a.inb, storage_id=""), rosbag2_py.ConverterOptions("", ""))
topics = r.get_all_topics_and_types()
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=a.outb, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
for t in topics:
    w.create_topic(t)
imu_cls = next((get_message(t.type) for t in topics if t.name == a.imu_topic), None)
t0, kept, dropped = None, 0, 0
while r.has_next():
    name, data, ts = r.read_next()
    t0 = ts if t0 is None else t0
    rel = (ts - t0) * 1e-9
    if name == a.imu_topic:
        if rnd.random() < a.drop_imu:
            dropped += 1; continue
        if a.imu_shift or a.imu_noise:
            m = deserialize_message(data, imu_cls)
            if a.imu_shift:
                ns = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec + int(a.imu_shift * 1e9)
                m.header.stamp.sec, m.header.stamp.nanosec = ns // 10**9, ns % 10**9
            if a.imu_noise:
                for f in ("angular_velocity", "linear_acceleration"):
                    v = getattr(m, f)
                    v.x += rnd.gauss(0, a.imu_noise); v.y += rnd.gauss(0, a.imu_noise); v.z += rnd.gauss(0, a.imu_noise)
            data = serialize_message(m)
    elif name == a.lidar_topic and blank and blank[0] <= rel <= blank[1]:
        dropped += 1; continue
    w.write(name, data, ts); kept += 1
print(f"wrote {a.outb}: kept {kept}, dropped {dropped}")

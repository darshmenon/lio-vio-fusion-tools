#!/usr/bin/env python3
"""Extract a nav_msgs/Odometry (or PoseStamped) topic from a ROS2 bag into a TUM trajectory file.
Usage: odom_to_tum.py <bag_dir> <topic> <out.tum>
Run with ROS2 sourced (needs rosbag2_py)."""
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main(bag, topic, out):
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=bag, storage_id="")
    conv = rosbag2_py.ConverterOptions("", "")
    reader.open(storage, conv)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f"topic {topic} not in bag. available: {list(types)}")
    msg_type = get_message(types[topic])
    n = 0
    with open(out, "w") as f:
        while reader.has_next():
            name, data, _ = reader.read_next()
            if name != topic:
                continue
            m = deserialize_message(data, msg_type)
            hdr = m.header
            pose = m.pose.pose if hasattr(m.pose, "pose") else m.pose
            t = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9
            p, q = pose.position, pose.orientation
            f.write(f"{t:.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
            n += 1
    print(f"wrote {n} poses to {out}")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])

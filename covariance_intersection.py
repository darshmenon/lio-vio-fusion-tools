#!/usr/bin/env python3
"""Covariance Intersection (CI) fusion of two odometry sources with UNKNOWN cross-correlation.

Why CI: LIO, VIO and the EKF share the same IMU/lidar, so their errors are correlated. A Kalman-style fusion that
assumes independence becomes overconfident; CI is consistent for any correlation:
    P^-1 = w P1^-1 + (1-w) P2^-1,    x = P (w P1^-1 x1 + (1-w) P2^-1 x2),    w in [0,1] minimising trace(P)
State = [position(3), rotation error(3)] around source 1's orientation. Poses must share a frame/origin.

Modes
  ROS node : covariance_intersection.py ros --a /kiss/odometry --b /vio/odom --out /ci/odom   (use_sim_time via ROS args)
             needs pose covariance on both inputs; run them through odom_health_gate.py / calibrate_covariance.py first
  offline  : covariance_intersection.py bag BAG --a TOPIC --b TOPIC --sigma-a POS ROT --sigma-b POS ROT --out fused.tum
             constant isotropic sigmas (e.g. from calibrate_covariance.py lio_sigma_pos/rot)
  selftest : covariance_intersection.py selftest
"""
import argparse
import sys

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation as R


def ci(x1, P1, x2, P2):
    """Fuse two Gaussian estimates; returns (x, P, w)."""
    I1, I2 = np.linalg.inv(P1), np.linalg.inv(P2)
    f = lambda w: np.trace(np.linalg.inv(w * I1 + (1 - w) * I2))
    w = float(minimize_scalar(f, bounds=(0.0, 1.0), method="bounded").x)
    P = np.linalg.inv(w * I1 + (1 - w) * I2)
    return P @ (w * I1 @ x1 + (1 - w) * I2 @ x2), P, w


def fuse_pose(p1, q1, C1, p2, q2, C2):
    """p: (3,), q: xyzw, C: 6x6 [pos, rot]. Returns fused (p, q, C, w)."""
    r1 = R.from_quat(q1)
    d = (r1.inv() * R.from_quat(q2)).as_rotvec()
    x, P, w = ci(np.zeros(6) + np.r_[p1, 0, 0, 0], C1, np.r_[p2, d], C2)
    return x[:3], (r1 * R.from_rotvec(x[3:])).as_quat(), P, w


def cov6(m):
    return np.array(m.pose.covariance).reshape(6, 6)


def selftest():
    rng = np.random.default_rng(0)
    # two sources with complementary uncertainty: CI must beat each alone and stay consistent under full correlation
    P1 = np.diag([0.01, 1.0, 1.0, .01, .01, .01]); P2 = np.diag([1.0, 0.01, 1.0, .01, .01, .01])
    x, P, w = ci(np.zeros(6), P1, np.zeros(6), P2)
    assert 0.2 < w < 0.8 and np.trace(P) < min(np.trace(P1), np.trace(P2)), (w, np.trace(P))
    # equal-quality sources: CI must not shrink below the single-source covariance (no double counting)
    Pe = np.eye(6) * 0.04; _, Pc, _ = ci(np.zeros(6), Pe, np.zeros(6), Pe)
    assert np.allclose(Pc, Pe, atol=1e-6), "CI must not double count identical information"
    print(f"selftest ok: complementary w={w:.2f}, trace {np.trace(P):.2f} < {min(np.trace(P1), np.trace(P2)):.2f}; identical sources keep P")


def run_bag(a):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {t: get_message(ty[t]) for t in (a.a, a.b)}
    A, B = [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n in cls:
            m = deserialize_message(d, cls[n]); o, p = m.pose.pose.orientation, m.pose.pose.position
            (A if n == a.a else B).append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])))
    tb = np.array([x[0] for x in B])
    Ca = np.diag([a.sigma_a[0] ** 2] * 3 + [a.sigma_a[1] ** 2] * 3); Cb = np.diag([a.sigma_b[0] ** 2] * 3 + [a.sigma_b[1] ** 2] * 3)
    ws = []
    with open(a.out, "w") as f:
        for t, p, q in A:
            j = int(np.abs(tb - t).argmin())
            if abs(tb[j] - t) > a.max_dt:
                continue
            pf, qf, _, w = fuse_pose(p, q, Ca, B[j][1], B[j][2], Cb); ws.append(w)
            f.write(f"{t:.9f} {pf[0]} {pf[1]} {pf[2]} {qf[0]} {qf[1]} {qf[2]} {qf[3]}\n")
    print(f"wrote {len(ws)} fused poses to {a.out}; mean CI weight on A = {np.mean(ws):.2f}")


def run_ros(a):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class N(Node):
        def __init__(s):
            super().__init__("covariance_intersection")
            s.b = None
            s.pub = s.create_publisher(Odometry, a.out, 10)
            s.create_subscription(Odometry, a.b, lambda m: setattr(s, "b", m), qos_profile_sensor_data)
            s.create_subscription(Odometry, a.a, s.cb, qos_profile_sensor_data)

        def cb(s, m):
            if s.b is None:
                return
            tm = lambda x: x.header.stamp.sec + x.header.stamp.nanosec * 1e-9
            if abs(tm(m) - tm(s.b)) > a.max_dt:
                return
            pa, pb = m.pose.pose.position, s.b.pose.pose.position; oa, ob = m.pose.pose.orientation, s.b.pose.pose.orientation
            p, q, C, w = fuse_pose([pa.x, pa.y, pa.z], [oa.x, oa.y, oa.z, oa.w], cov6(m), [pb.x, pb.y, pb.z], [ob.x, ob.y, ob.z, ob.w], cov6(s.b))
            o = Odometry(); o.header = m.header; o.child_frame_id = m.child_frame_id
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
            o.pose.pose.orientation.x, o.pose.pose.orientation.y, o.pose.pose.orientation.z, o.pose.pose.orientation.w = map(float, q)
            o.pose.covariance = C.flatten().tolist(); s.pub.publish(o)

    rclpy.init(); rclpy.spin(N())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["ros", "bag", "selftest"]); ap.add_argument("bag", nargs="?")
    ap.add_argument("--a"); ap.add_argument("--b"); ap.add_argument("--out")
    ap.add_argument("--sigma-a", type=float, nargs=2, default=[0.05, 0.02]); ap.add_argument("--sigma-b", type=float, nargs=2, default=[0.05, 0.02])
    ap.add_argument("--max-dt", type=float, default=0.1)
    a = ap.parse_args()
    {"selftest": lambda _: selftest(), "bag": run_bag, "ros": run_ros}[a.mode](a)


if __name__ == "__main__":
    main()

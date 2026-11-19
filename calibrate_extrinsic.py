#!/usr/bin/env python3
"""Estimate the IMU -> odometry-body rotation extrinsic and time offset from a bag (gyro vs LIO/VIO/GT angular velocity).

Method: angular velocity of the body measured two ways, w_imu(t) from the gyro and w_odo(t) from successive odometry
orientations (body frame). They satisfy w_odo(t) = R w_imu(t + dt). dt is found by cross-correlation of |w| (coarse) and refined
on the full vectors; R by Kabsch/SVD (Wahba). The singular values of the cross-covariance say which rotation axes were excited:
a ground robot that only yaws constrains one axis (yaw misalignment) - the tool says so instead of inventing roll/pitch.

Usage (ROS2 sourced): calibrate_extrinsic.py BAG IMU_TOPIC ODOM_TOPIC [--max-offset 0.3]
       calibrate_extrinsic.py selftest
Translation (lever arm) and camera-IMU extrinsics are NOT estimated here: use Kalibr (camera-IMU) / LiDAR_IMU_Init (lidar-IMU).
"""
import argparse
import sys

import numpy as np
from scipy.spatial.transform import Rotation as R


def kabsch(a, b):
    """R minimising sum |b - R a|^2 for 3xN-like rows (N,3). Returns R, singular values."""
    H = a.T @ b
    u, s, vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1, 1, d]) @ u.T, s


def odo_rate(t, q):
    r = R.from_quat(q)
    w = (r[:-1].inv() * r[1:]).as_rotvec() / np.diff(t)[:, None]   # body-frame angular velocity
    return (t[:-1] + t[1:]) / 2, w


def estimate(ti, wi, to, wo, max_off=0.3, dt_step=0.005):
    grid = np.arange(max(ti[0], to[0]) + 1, min(ti[-1], to[-1]) - 1, dt_step)
    A = np.stack([np.interp(grid, ti, wi[:, k]) for k in range(3)], 1)
    best = (-2, 0.0, None)
    for k in range(-int(max_off / dt_step), int(max_off / dt_step) + 1):
        B = np.stack([np.interp(grid + k * dt_step, to, wo[:, c]) for c in range(3)], 1)
        a, b = np.linalg.norm(A, axis=1), np.linalg.norm(B, axis=1)
        c = np.corrcoef(a, b)[0, 1]
        if c > best[0]:
            best = (c, k * dt_step, B)
    corr, off, B = best
    Rm, s = kabsch(A, B)
    u, _, vt = np.linalg.svd(A.T @ B); axes = (u[:, 0], vt[0])   # dominant rotation axis seen by the IMU / by the odometry
    resid = np.sqrt(((B - A @ Rm.T) ** 2).sum(1).mean()); scale = np.sqrt((B ** 2).sum(1).mean()) + 1e-12
    return off, corr, Rm, s, resid / scale, axes


def describe(off, corr, Rm, s, rel, axes=None):
    ypr = R.from_matrix(Rm).as_euler("ZYX", degrees=True)
    ax = int((s > 0.05 * s[0]).sum())
    print(f"time offset (odo stamps lag IMU): {off:+.3f} s   |w| correlation {corr:.3f}")
    ax = int((s > 0.05 * s[0]).sum())
    if ax >= 3:
        ypr = R.from_matrix(Rm).as_euler("ZYX", degrees=True)
        print(f"rotation IMU->odometry body (yaw,pitch,roll deg): {ypr[0]:+.2f} {ypr[1]:+.2f} {ypr[2]:+.2f}")
    elif axes is not None:
        a, b = axes; c = float(np.clip(abs(a @ b), 0, 1))
        print(f"only ONE rotation axis excited: mount TILT (angle between the IMU's and the odometry's turning axis) = {np.degrees(np.arccos(c)):.2f} deg")
        print("  rotation ABOUT that axis (e.g. yaw misalignment of a ground robot) is NOT observable from this motion; excite pitch/roll too, or use LiDAR_IMU_Init / Kalibr.")
    print(f"excited rotation axes: {ax}/3 (singular values {s[0]:.1f} {s[1]:.2f} {s[2]:.2f}); relative residual {rel:.2f}" + ("  (high residual: gyro vs odometry disagree, e.g. gait vibration; do not trust)" if rel > 0.5 else ""))


def selftest():
    rng = np.random.default_rng(1)
    t = np.arange(0, 60, 0.005); Rtrue = R.from_euler("ZYX", [10, -5, 3], degrees=True).as_matrix(); off = 0.04
    w_body = np.stack([np.sin(1.3 * t), np.cos(0.9 * t), 0.7 * np.sin(0.5 * t + 1)], 1) * 1.2
    q = [R.identity()]
    for k in range(len(t) - 1):  # proper body-frame integration: R_{k+1} = R_k * exp(w_k dt)
        q.append(q[-1] * R.from_rotvec(w_body[k] * 0.005))
    ori = R.concatenate(q)
    to = t[::10] + off; qo = ori[::10].as_quat()
    wi = (w_body @ np.linalg.inv(Rtrue).T) + rng.normal(0, 0.01, w_body.shape)  # IMU measures R^-1 w
    odo_t, odo_w = odo_rate(to, qo)
    o, c, Rm, s, rel, axes = estimate(t, wi, odo_t, odo_w)
    err = np.degrees(np.linalg.norm(R.from_matrix(Rm @ Rtrue.T).as_rotvec()))
    describe(o, c, Rm, s, rel, axes)
    assert abs(o - off) < 0.011 and err < 1.5, (o, err)
    print(f"selftest ok: recovered offset {o:+.3f} s (true {off:+.3f}), rotation error {err:.2f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag"); ap.add_argument("imu", nargs="?"); ap.add_argument("odom", nargs="?"); ap.add_argument("--max-offset", type=float, default=0.3)
    a = ap.parse_args()
    if a.bag == "selftest":
        return selftest()
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {x: get_message(ty[x]) for x in (a.imu, a.odom)}
    I, O = [], []
    while r.has_next():
        n, d, _ = r.read_next()
        if n in cls:
            m = deserialize_message(d, cls[n]); st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if n == a.imu:
                I.append((st, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
            else:
                p = m.pose.pose if hasattr(m.pose, "pose") else m.pose; o = p.orientation
                O.append((st, o.x, o.y, o.z, o.w))
    I, O = np.array(I), np.array(O)
    to, wo = odo_rate(O[:, 0], O[:, 1:5])
    describe(*estimate(I[:, 0], I[:, 1:4], to, wo, a.max_offset))


if __name__ == "__main__":
    main()

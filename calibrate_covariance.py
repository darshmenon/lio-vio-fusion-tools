#!/usr/bin/env python3
"""Estimate honest odometry covariances from a recorded bag + ground truth, write a params file for odom_health_gate.py.

For each source it measures the error of its body-frame motion against ground truth:
  wheel : forward speed and yaw rate (twist) vs GT-derived body velocity  -> wheel_sigma_v / wheel_sigma_w (+ bias, scale)
  LIO   : translation / rotation error over 1 s windows (RPE-style)        -> lio_sigma_pos / lio_sigma_rot
Wheel sigmas are the total RMS error (bias and scale error included, since an EKF treats them as noise; the robust std
is printed too and can hide a large systematic bias); the gate's adaptive term handles slip bursts online. Use --before FRAC to calibrate on the first part of the bag only (hold out the rest).

Usage (ROS2 sourced): calibrate_covariance.py --bag B --gt /gt_odom --wheel /odom --lio /kiss/odometry --out gate_params.yaml [--before 0.5]
"""
import argparse
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R


def read(bag, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {t: get_message(ty[t]) for t in topics}
    out = {t: [] for t in topics}
    while r.has_next():
        n, d, _ = r.read_next()
        if n in out:
            m = deserialize_message(d, cls[n]); o = m.pose.pose.orientation; p = m.pose.pose.position
            out[n].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, [p.x, p.y, p.z], [o.x, o.y, o.z, o.w],
                           m.twist.twist.linear.x, m.twist.twist.angular.z))
    return {k: (np.array([x[0] for x in v]), np.array([x[1] for x in v]), np.array([x[2] for x in v]),
                np.array([x[3] for x in v]), np.array([x[4] for x in v])) for k, v in out.items()}


def robust_std(x):
    return 1.4826 * np.median(np.abs(x - np.median(x)))


def body_vel(t, p, q, win=0.3):
    """Body-frame forward speed and yaw rate from poses, differenced over ~win seconds."""
    j = np.searchsorted(t, t + win).clip(max=len(t) - 1)
    ok = (t[j] - t) > win * 0.5
    dt = (t[j] - t)[ok]; idx = np.nonzero(ok)[0]; jj = j[ok]
    r0 = R.from_quat(q[idx])
    v = r0.inv().apply(p[jj] - p[idx])[:, 0] / dt
    w = (r0.inv() * R.from_quat(q[jj])).as_rotvec()[:, 2] / dt
    return t[idx] + dt / 2, v, w


def rpe_1s(t, p, q, tg, pg, qg):
    j = np.searchsorted(t, t + 1.0).clip(max=len(t) - 1); ok = (t[j] - t) > 0.8
    idx = np.nonzero(ok)[0]; jj = j[ok]
    def at(tt, arrp, arrq):  # GT nearest sample
        k = np.abs(tg[None, :] - tt[:, None]).argmin(1) if len(tt) < 4000 else np.searchsorted(tg, tt).clip(0, len(tg) - 1)
        return arrp[k], arrq[k]
    pa, qa = at(t[idx], pg, qg); pb, qb = at(t[jj], pg, qg)
    rel_e = R.from_quat(q[idx]).inv().apply(p[jj] - p[idx]); rel_g = R.from_quat(qa).inv().apply(pb - pa)
    te = np.linalg.norm(rel_e - rel_g, axis=1)
    re = (R.from_quat(q[idx]).inv() * R.from_quat(q[jj])); rg = (R.from_quat(qa).inv() * R.from_quat(qb))
    ae = np.linalg.norm((rg.inv() * re).as_rotvec(), axis=1)
    return te, ae


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True); ap.add_argument("--gt", required=True)
    ap.add_argument("--wheel", required=True); ap.add_argument("--lio", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--before", type=float, default=1.0)
    a = ap.parse_args()
    d = read(a.bag, [a.gt, a.wheel, a.lio])
    tg, pg, qg, _, _ = d[a.gt]
    cut = tg[0] + a.before * (tg[-1] - tg[0])
    sel = lambda t: t <= cut
    gm = sel(tg); tg, pg, qg = tg[gm], pg[gm], qg[gm]
    tv, vg, wg = body_vel(tg, pg, qg)
    tw, _, _, vw, ww = d[a.wheel]; m = sel(tw) & (tw >= tv[0]) & (tw <= tv[-1]); tw, vw, ww = tw[m], vw[m], ww[m]
    vgi, wgi = np.interp(tw, tv, vg), np.interp(tw, tv, wg)
    ev, ew = vw - vgi, ww - wgi
    tl, pl, ql, _, _ = d[a.lio]; m = sel(tl); tl, pl, ql = tl[m], pl[m], ql[m]
    te, ae = rpe_1s(tl, pl, ql, tg, pg, qg)
    moving = np.abs(vgi) > 0.05
    res = dict(
        wheel_sigma_v=float(max(np.sqrt((ev ** 2).mean()), 0.01)),  # all samples: the EKF also sees the wheel while GT is still
        wheel_sigma_w=float(max(np.sqrt((ew ** 2).mean()), 0.01)),
        lio_sigma_pos=float(max(np.sqrt((te ** 2).mean()), 0.02)),
        lio_sigma_rot=float(max(np.sqrt((ae ** 2).mean()), 0.01)))
    scale = float(np.polyfit(vgi[moving], vw[moving], 1)[0]) if moving.sum() > 10 else float("nan")
    print(f"calibrated on t <= {a.before:.0%} of bag ({len(tw)} wheel, {len(tl)} lio msgs)")
    print(f"  wheel: robust sigma v {res['wheel_sigma_v']:.3f} m/s, w {res['wheel_sigma_w']:.3f} rad/s | rms v {np.sqrt((ev**2).mean()):.3f}, w {np.sqrt((ew**2).mean()):.3f} | speed scale {scale:.3f}, bias v {np.mean(ev):+.3f} w {np.mean(ew):+.3f}")
    print(f"  lio  : 1 s window rms trans {res['lio_sigma_pos']:.3f} m, rot {res['lio_sigma_rot']:.3f} rad")
    with open(a.out, "w") as f:
        f.write("# generated by calibrate_covariance.py\nodom_health_gate:\n  ros__parameters:\n")
        for k, v in res.items():
            f.write(f"    {k}: {v:.5f}\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()

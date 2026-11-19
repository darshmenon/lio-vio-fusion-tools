#!/usr/bin/env python3
"""One-step LIO/VIO report from a ROS2 bag: APE/RPE table + trajectory and error plots.

Usage (ROS2 sourced):
  report.py --bag BAG --gt /gt_odom --est kiss=/kiss/odometry --est rko=/rko_lio/odom --out OUT [--tmax 0.05] [--title T]
  An estimate may live in another bag: --est rko=/rko_lio/odom@/path/to/est_bag

Writes OUT/report.md, OUT/traj.png, OUT/ape_time.png and one <name>.tum per trajectory.
Each estimate is matched to ground truth by nearest timestamp (within --tmax) and SE3-aligned
(Umeyama, no scale), the same convention as `evo_ape -a`.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from odom_to_tum import main as extract


def load(path, tmin=None):
    a = np.loadtxt(path)
    if tmin is not None:
        a = a[a[:, 0] >= tmin]
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def match(t_ref, t_est, tmax):
    idx = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    left = np.where(np.abs(t_ref[idx - 1] - t_est) < np.abs(t_ref[idx] - t_est), idx - 1, idx)
    ok = np.abs(t_ref[left] - t_est) <= tmax
    return left[ok], np.nonzero(ok)[0]


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((dst - mu_d).T @ (src - mu_s))
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    rot = u @ s @ vt
    return rot, mu_d - rot @ mu_s


def rpe(pg, qg, pe, qe, delta=1.0):
    """Translation RPE over segments of `delta` metres of ground-truth path."""
    d = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))])
    errs, j = [], 0
    for i in range(len(pg)):
        while j < len(pg) and d[j] - d[i] < delta:
            j += 1
        if j >= len(pg):
            break
        rg_i, rg_j = R.from_quat(qg[i]), R.from_quat(qg[j])
        re_i, re_j = R.from_quat(qe[i]), R.from_quat(qe[j])
        rel_g = rg_i.inv().apply(pg[j] - pg[i])
        rel_e = re_i.inv().apply(pe[j] - pe[i])
        errs.append(np.linalg.norm(rel_g - rel_e))
    return np.array(errs)


def score(gt, est, tmax):
    tg, pg, qg = gt
    te, pe, qe = est
    gi, ei = match(tg, te, tmax)
    if len(gi) < 10:
        return None
    pg, qg, pe, qe, tt = pg[gi], qg[gi], pe[ei], qe[ei], te[ei]
    rot, tr = umeyama(pe, pg)
    pa = (rot @ pe.T).T + tr
    err = np.linalg.norm(pa - pg, axis=1)
    path = np.linalg.norm(np.diff(pg, axis=0), axis=1).sum()
    r = rpe(pg, qg, pe, qe)
    return dict(n=len(gi), path=path, ape_rmse=np.sqrt((err ** 2).mean()), ape_mean=err.mean(), ape_max=err.max(),
                end_drift=err[-1], drift_pct=100 * err[-1] / max(path, 1e-9),
                rpe_rmse=np.sqrt((r ** 2).mean()) if len(r) else float("nan"), pa=pa, pg=pg, t=tt, err=err)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--gt", required=True)
    ap.add_argument("--est", action="append", required=True, help="name=topic (repeatable)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tmax", type=float, default=0.05)
    ap.add_argument("--title", default="")
    ap.add_argument("--after", type=float, default=0.0, help="score only the last part of the bag: poses after this fraction of ground-truth time (hold-out)")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    extract(a.bag, a.gt, os.path.join(a.out, "gt.tum"))
    gt = load(os.path.join(a.out, "gt.tum"))
    tmin = gt[0][0] + a.after * (gt[0][-1] - gt[0][0]) if a.after > 0 else None
    gt = load(os.path.join(a.out, "gt.tum"), tmin)
    res = {}
    for spec in a.est:
        name, topic = spec.split("=", 1)
        bag = a.bag
        if "@" in topic:  # name=topic@other_bag: estimate lives in a different bag (e.g. offline replay)
            topic, bag = topic.split("@", 1)
        f = os.path.join(a.out, f"{name}.tum")
        if topic.endswith(".tum"):  # name=path.tum: use a ready trajectory file
            f = topic
        else:
            extract(bag, topic, f)
        est = load(f, tmin) if os.path.getsize(f) else None
        res[name] = score(gt, est, a.tmax) if est is not None and len(est[0]) > 1 else None

    lines = [f"# LIO/VIO report {a.title}".rstrip(), "", f"bag: `{a.bag}`  ground truth: `{a.gt}`  match tol: {a.tmax} s", "",
             "| method | matched | path (m) | APE RMSE (m) | APE mean | APE max | end drift (m) | drift % | RPE 1 m RMSE (m) |",
             "|---|---|---|---|---|---|---|---|---|"]
    for n, s in res.items():
        if s is None:
            lines.append(f"| {n} | no usable poses | | | | | | | |")
        else:
            lines.append(f"| {n} | {s['n']} | {s['path']:.1f} | {s['ape_rmse']:.3f} | {s['ape_mean']:.3f} | {s['ape_max']:.3f} "
                         f"| {s['end_drift']:.3f} | {s['drift_pct']:.2f} | {s['rpe_rmse']:.3f} |")
    open(os.path.join(a.out, "report.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))

    good = {n: s for n, s in res.items() if s}
    if good:
        fig, ax = plt.subplots(figsize=(6, 6))
        first = next(iter(good.values()))
        ax.plot(first["pg"][:, 0], first["pg"][:, 1], "k--", lw=2, label="ground truth")
        for n, s in good.items():
            ax.plot(s["pa"][:, 0], s["pa"][:, 1], lw=1.2, label=n)
        ax.set_aspect("equal"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.legend(); ax.grid(alpha=.3)
        ax.set_title("Trajectories (SE3-aligned) " + a.title)
        fig.savefig(os.path.join(a.out, "traj.png"), dpi=130, bbox_inches="tight")
        fig, ax = plt.subplots(figsize=(8, 3.5))
        for n, s in good.items():
            ax.plot(s["t"] - s["t"][0], s["err"], lw=1.2, label=n)
        ax.set_xlabel("time (s)"); ax.set_ylabel("APE (m)"); ax.legend(); ax.grid(alpha=.3)
        ax.set_title("Position error over time " + a.title)
        fig.savefig(os.path.join(a.out, "ape_time.png"), dpi=130, bbox_inches="tight")


if __name__ == "__main__":
    main()

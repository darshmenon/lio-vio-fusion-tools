#!/usr/bin/env python3
"""Odometry health gate for robot_localization: sets honest covariances and down-weights a source that misbehaves.

Why: sim/real wheel odometry often publishes ZERO covariance, so an EKF trusts it absolutely and slip drags the
fused result below the LIO alone. This node sits between the sources and the EKF:
  * wheel: variance is inflated by how much the wheel body velocity disagrees with the LIO-derived velocity
    (EMA of the residual, so one noisy sample does not matter, but sustained slip does).
  * LIO: a jump (implied speed > max_speed) marks it untrusted for hold_s (a stall > stale_s only resets the velocity estimate, the
    pose after an outage is still valid): its pose covariance is set huge so the EKF leans on wheel+IMU until it recovers (the failure a
    diverged LIO-SAM shows). Healthy LIO gets a fixed sigma.
Params (ros2 -p): lio_in wheel_in lio_out wheel_out, lio_sigma_pos/rot, wheel_sigma_v/w, tol_v, tol_w,
                  max_speed, stale_s, hold_s, ema_alpha, adaptive (default true: variance = EMA of squared residual, floored at sigma).   Use with use_sim_time:=true when replaying a bag.
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R


class Gate(Node):
    def __init__(self):
        super().__init__("odom_health_gate")
        P = self.declare_parameter
        P("lio_in", "/kiss/odometry"); P("wheel_in", "/odom"); P("lio_out", "/gated/lio"); P("wheel_out", "/gated/wheel")
        P("lio_sigma_pos", 0.05); P("lio_sigma_rot", 0.05); P("wheel_sigma_v", 0.05); P("wheel_sigma_w", 0.05)
        P("tol_v", 0.05); P("tol_w", 0.05); P("max_speed", 3.0); P("stale_s", 1.0); P("hold_s", 2.0); P("ema_alpha", 0.05); P("adaptive", True)
        g = lambda k: self.get_parameter(k).value
        self.p = {k: g(k) for k in ("lio_sigma_pos", "lio_sigma_rot", "wheel_sigma_v", "wheel_sigma_w", "tol_v", "tol_w",
                                    "max_speed", "stale_s", "hold_s", "ema_alpha")}
        self.adaptive = g("adaptive"); self.ev2 = 0.0; self.ew2 = 0.0
        self.pub_l = self.create_publisher(Odometry, g("lio_out"), 10)
        self.pub_w = self.create_publisher(Odometry, g("wheel_out"), 10)
        self.create_subscription(Odometry, g("lio_in"), self.on_lio, qos_profile_sensor_data)
        self.create_subscription(Odometry, g("wheel_in"), self.on_wheel, qos_profile_sensor_data)
        self.prev = None; self.v_l = None; self.w_l = None; self.t_l = -1e9
        self.bad_until = -1e9; self.rv = 0.0; self.rw = 0.0
        self.n_jump = 0; self.n_lio = 0; self.n_w = 0
        self.create_timer(10.0, self.report)

    def now_s(self, m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_lio(self, m):
        t = self.now_s(m); o = m.pose.pose.orientation; pp = m.pose.pose.position
        pos = np.array([pp.x, pp.y, pp.z]); rot = R.from_quat([o.x, o.y, o.z, o.w])
        if self.prev is not None:
            t0, p0, r0 = self.prev; dt = t - t0
            if dt > 1e-3:
                rel = r0.inv().apply(pos - p0) / dt
                yaw = (r0.inv() * rot).as_rotvec()[2] / dt
                speed = float(np.linalg.norm(pos - p0)) / dt
                if dt > self.p["stale_s"]:
                    # stall: the pose after an outage is still valid, so no hold; just forget the stale velocity
                    self.v_l = self.w_l = None; self.n_stall = getattr(self, "n_stall", 0) + 1
                elif speed > self.p["max_speed"]:
                    self.bad_until = t + self.p["hold_s"]; self.n_jump += 1
                else:
                    a = 0.3  # light smoothing of LIO body velocity
                    self.v_l = rel[0] if self.v_l is None else (1 - a) * self.v_l + a * rel[0]
                    self.w_l = yaw if self.w_l is None else (1 - a) * self.w_l + a * yaw
                    self.t_l = t
        self.prev = (t, pos, rot); self.n_lio += 1
        out = Odometry(); out.header = m.header; out.child_frame_id = m.child_frame_id; out.pose.pose = m.pose.pose
        out.twist = m.twist
        trusted = t >= self.bad_until
        sp = self.p["lio_sigma_pos"] if trusted else 1e3; sr = self.p["lio_sigma_rot"] if trusted else 1e3
        c = [0.0] * 36
        for i in range(3): c[i * 7] = sp ** 2
        for i in range(3, 6): c[i * 7] = sr ** 2
        out.pose.covariance = c
        self.pub_l.publish(out)

    def on_wheel(self, m):
        t = self.now_s(m); a = self.p["ema_alpha"]
        lio_fresh = self.v_l is not None and abs(t - self.t_l) < 0.5 and t >= self.bad_until
        if lio_fresh:
            dv, dw = m.twist.twist.linear.x - self.v_l, m.twist.twist.angular.z - self.w_l
            self.rv = (1 - a) * self.rv + a * abs(dv); self.rw = (1 - a) * self.rw + a * abs(dw)
            self.ev2 = (1 - a) * self.ev2 + a * dv * dv; self.ew2 = (1 - a) * self.ew2 + a * dw * dw
        # no fresh trusted LIO -> nothing to compare against: keep the last residual-based trust (do not reward or punish)
        if self.adaptive:  # adaptive variance: running mean of the squared wheel-vs-LIO disagreement, floored at the calibrated sigma
            fv = max(1.0, self.ev2 / self.p["wheel_sigma_v"] ** 2); fw = max(1.0, self.ew2 / self.p["wheel_sigma_w"] ** 2)
        else:              # legacy ratio heuristic
            fv = 1.0 + (self.rv / self.p["tol_v"]) ** 2; fw = 1.0 + (self.rw / self.p["tol_w"]) ** 2
        out = Odometry(); out.header = m.header; out.child_frame_id = m.child_frame_id
        out.pose = m.pose; out.twist.twist = m.twist.twist
        c = [0.0] * 36
        c[0] = (self.p["wheel_sigma_v"] ** 2) * fv; c[7] = 0.1; c[14] = 1e3
        c[21] = c[28] = 1e3; c[35] = (self.p["wheel_sigma_w"] ** 2) * fw
        out.twist.covariance = c
        pc = [0.0] * 36
        for i in range(6): pc[i * 7] = 1e3  # pose from wheel is not used; make that explicit
        out.pose.covariance = pc
        self.n_w += 1; self.fv, self.fw = fv, fw
        self.pub_w.publish(out)

    def report(self):
        self.get_logger().info(f"lio msgs {self.n_lio} jumps/stalls {self.n_jump} | wheel msgs {self.n_w} "
                               f"resid v {self.rv:.3f} w {self.rw:.3f} m/s,rad/s -> wheel var x{getattr(self, 'fv', 1):.1f}/{getattr(self, 'fw', 1):.1f}")


def main():
    rclpy.init(); n = Gate()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

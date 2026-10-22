"""Differential-drive kinematics, rate limiting and encoder odometry (same math in sim and on the robot)."""
from __future__ import annotations

import math


def twist_to_wheels(v: float, w: float, radius: float, separation: float) -> tuple[float, float]:
    """Body twist -> wheel angular velocities (rad/s), positive = forward on both wheels."""
    return (v - w * separation / 2) / radius, (v + w * separation / 2) / radius


def wheels_to_twist(wl: float, wr: float, radius: float, separation: float) -> tuple[float, float]:
    return radius * (wl + wr) / 2, radius * (wr - wl) / separation


class RateLimiter:
    """Velocity + acceleration limits (matches diff_drive_controller limits)."""

    def __init__(self, v_max, w_max, a_max, alpha_max):
        self.v_max, self.w_max, self.a_max, self.alpha_max = v_max, w_max, a_max, alpha_max
        self.v = 0.0
        self.w = 0.0

    def __call__(self, v_cmd: float, w_cmd: float, dt: float) -> tuple[float, float]:
        v_cmd = max(-self.v_max, min(self.v_max, v_cmd))
        w_cmd = max(-self.w_max, min(self.w_max, w_cmd))
        dv = max(-self.a_max * dt, min(self.a_max * dt, v_cmd - self.v))
        dw = max(-self.alpha_max * dt, min(self.alpha_max * dt, w_cmd - self.w))
        self.v += dv
        self.w += dw
        return self.v, self.w


class EncoderOdometry:
    """Integrates wheel angle increments (from encoders) into an odom-frame pose."""

    def __init__(self, radius: float, separation: float, x=0.0, y=0.0, yaw=0.0):
        self.r, self.b = radius, separation
        self.x, self.y, self.yaw = x, y, yaw
        self.prev = None
        self.v = self.w = 0.0

    def update(self, ang_l: float, ang_r: float, dt: float):
        if self.prev is None:
            self.prev = (ang_l, ang_r)
            return self.x, self.y, self.yaw
        dl = (ang_l - self.prev[0]) * self.r
        dr = (ang_r - self.prev[1]) * self.r
        self.prev = (ang_l, ang_r)
        ds, dth = (dl + dr) / 2, (dr - dl) / self.b
        self.x += ds * math.cos(self.yaw + dth / 2)
        self.y += ds * math.sin(self.yaw + dth / 2)
        self.yaw = math.atan2(math.sin(self.yaw + dth), math.cos(self.yaw + dth))
        if dt > 0:
            self.v, self.w = ds / dt, dth / dt
        return self.x, self.y, self.yaw


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

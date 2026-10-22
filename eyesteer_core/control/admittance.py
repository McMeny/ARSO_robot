"""Walker handle admittance: handle forces -> velocity (ASSIST mode) and pull-back scaling (ENGAGED).

Forces are measured along the walker's +x (push forward > 0). Torque: tau = y_h (F_right - F_left)
(pushing the right handle harder turns left, like a rollator).
"""
from __future__ import annotations

import math


def deadband(f: float, db: float) -> float:
    if abs(f) <= db:
        return 0.0
    return f - math.copysign(db, f)


class Admittance:
    def __init__(self, mass=25.0, damping=40.0, yaw_inertia=4.0, yaw_damping=8.0, force_deadband=4.0,
                 handle_y=0.25, v_max=0.5, w_max=0.8):
        self.m, self.b = mass, damping
        self.J, self.bw = yaw_inertia, yaw_damping
        self.db = force_deadband
        self.hy = handle_y
        self.v_max, self.w_max = v_max, w_max
        self.v = 0.0
        self.w = 0.0

    def reset(self):
        self.v = self.w = 0.0

    def step(self, f_left: float, f_right: float, dt: float) -> tuple[float, float]:
        fl, fr = deadband(f_left, self.db), deadband(f_right, self.db)
        F = fl + fr
        tau = self.hy * (fr - fl)
        self.v += dt * (F - self.b * self.v) / self.m
        self.w += dt * (tau - self.bw * self.w) / self.J
        self.v = max(-self.v_max * 0.5, min(self.v_max, self.v))   # walking backwards is slower
        self.w = max(-self.w_max, min(self.w_max, self.w))
        return self.v, self.w

    def active(self, f_left: float, f_right: float) -> bool:
        return abs(f_left) > self.db or abs(f_right) > self.db or abs(self.v) > 0.02 or abs(self.w) > 0.02


def pullback_scale(f_left: float, f_right: float, threshold: float = 12.0, stop: float = 30.0) -> float:
    """1.0 = nav at full speed; 0.0 = stopped. Pulling back (negative net force) beyond `threshold`
    scales the nav command down linearly, reaching zero at `stop`. The user always wins."""
    pull = -(f_left + f_right)
    if pull <= threshold:
        return 1.0
    if pull >= stop:
        return 0.0
    return 1.0 - (pull - threshold) / (stop - threshold)

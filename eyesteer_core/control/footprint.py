"""Footprints (walker includes the user, D10), transforms, clearance and collision-monitor logic."""
from __future__ import annotations

import math

import numpy as np


class Footprint:
    def __init__(self, fp: dict):
        """fp: generated footprint dict ({type: circle, radius} or {type: polygon, polygon})."""
        self.type = fp["type"]
        self.radius = fp["radius"]
        self.padding = fp.get("padding", 0.0)
        if self.type == "polygon":
            self.poly = np.array(fp["polygon"], float)
        else:
            a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            self.poly = np.stack([self.radius * np.cos(a), self.radius * np.sin(a)], 1)

    def grown(self, d: float) -> np.ndarray:
        """Polygon expanded by d (local frame)."""
        if self.type == "circle":
            a = np.linspace(0, 2 * np.pi, 24, endpoint=False)
            r = self.radius + d
            return np.stack([r * np.cos(a), r * np.sin(a)], 1)
        return np.array([[x + math.copysign(d, x), y + math.copysign(d, y)] for x, y in self.poly])

    def world(self, x, y, yaw, poly=None) -> np.ndarray:
        p = self.poly if poly is None else poly
        c, s = math.cos(yaw), math.sin(yaw)
        return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], 1)

    def sample_points(self, x, y, yaw, spacing=0.05) -> np.ndarray:
        """Boundary samples (for clearance against an SDF)."""
        if self.type == "circle":
            return self.world(x, y, yaw)
        w = self.world(x, y, yaw)
        pts = []
        for i in range(len(w)):
            a, b = w[i], w[(i + 1) % len(w)]
            n = max(2, int(np.linalg.norm(b - a) / spacing))
            pts.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
        return np.concatenate(pts)

    def clearance(self, x, y, yaw, sdf_fn, circles=()) -> float:
        """Min distance from the footprint boundary to obstacles (negative = penetration).

        sdf_fn(px, py) -> static signed distance; circles: [(cx, cy, r)] for moving people.
        """
        if self.type == "circle":
            d = float(sdf_fn(np.array([x]), np.array([y]))[0]) - self.radius
            for cx, cy, r in circles:
                d = min(d, math.hypot(cx - x, cy - y) - r - self.radius)
            return d
        pts = self.sample_points(x, y, yaw)
        d = float(np.min(sdf_fn(pts[:, 0], pts[:, 1])))
        inside_static = False
        for cx, cy, r in circles:
            dd = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r
            dm = float(dd.min())
            if point_in_poly(np.array([[cx, cy]]), self.world(x, y, yaw))[0]:
                dm = -r
            d = min(d, dm)
        return d if not inside_static else -1.0


def point_in_poly(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Vectorized even-odd rule. pts (n, 2), poly (m, 2)."""
    x, y = pts[:, 0][:, None], pts[:, 1][:, None]
    x0, y0 = poly[:, 0][None], poly[:, 1][None]
    x1, y1 = np.roll(poly[:, 0], -1)[None], np.roll(poly[:, 1], -1)[None]
    cond = (y0 > y) != (y1 > y)
    xint = x0 + (y - y0) * (x1 - x0) / np.where(y1 - y0 == 0, 1e-12, y1 - y0)
    return (cond & (x < xint)).sum(1) % 2 == 1


class CollisionMonitor:
    """Mirror of the Nav2 Collision Monitor config: slowdown polygon (+0.4 m -> 30 %), stop polygon (+0.1 m).

    Input: LiDAR points in the base frame. Output: (scale, state) with state in {OK, SLOWDOWN, STOP}.
    """

    def __init__(self, fp_dict: dict, slow_margin=0.4, slow_ratio=0.3, stop_margin=0.1, min_points=2):
        self.slow_poly = np.array(monitor_polygon(fp_dict, slow_margin))
        self.stop_poly = np.array(monitor_polygon(fp_dict, stop_margin))
        self.slow_ratio = slow_ratio
        self.min_points = min_points
        self.state = "OK"
        self.activations = 0

    def __call__(self, pts_base: np.ndarray, v: float = 0.0) -> tuple[float, str]:
        if len(pts_base) == 0:
            new = "OK"
        else:
            if point_in_poly(pts_base, self.stop_poly).sum() >= self.min_points:
                new = "STOP"
            elif point_in_poly(pts_base, self.slow_poly).sum() >= self.min_points:
                new = "SLOWDOWN"
            else:
                new = "OK"
        if new != "OK" and new != self.state:
            self.activations += 1
        self.state = new
        return {"OK": 1.0, "SLOWDOWN": self.slow_ratio, "STOP": 0.0}[new], new


class RotationGuard:
    """In-place rotation sweeps the whole footprint (the walker's user swings behind it), but the
    collision-monitor zones only look ahead and the rear LiDAR sector is occluded by the user. So a
    short memory of LiDAR points (world frame) is checked against the full footprint grown by `margin`
    before an in-place turn is allowed."""

    def __init__(self, fp: Footprint, margin: float = 0.1, memory_s: float = 2.0):
        self.poly = fp.grown(margin)
        self.memory_s = memory_s
        self.mem: list = []

    def add_scan(self, t: float, pts_world: np.ndarray):
        self.mem.append((t, pts_world))
        self.mem = [(tt, p) for tt, p in self.mem if t - tt <= self.memory_s]

    def blocked(self, x: float, y: float, yaw: float, extra_pts: np.ndarray | None = None) -> bool:
        pts = [p for _, p in self.mem if len(p)]
        if extra_pts is not None and len(extra_pts):
            pts.append(extra_pts)
        if not pts:
            return False
        P = np.concatenate(pts)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = P[:, 0] - x, P[:, 1] - y
        local = np.stack([c * dx + s * dy, -s * dx + c * dy], 1)
        return int(point_in_poly(local, self.poly).sum()) >= 2


def monitor_polygon(fp: dict, margin: float, side_margin: float | None = None) -> list:
    """Collision-monitor zone in the base frame: the FRONT half of the footprint, extended `margin`
    ahead and `side_margin` (default margin / 4) to the sides.

    Front-only so an obstacle beside/behind the robot (wall in an aisle, the walker user) can never
    deadlock it; narrow at the sides so aisle walls do not throttle the robot. The same polygon is
    written into the Nav2 collision_monitor params.
    """
    side = margin / 4 if side_margin is None else side_margin
    if fp["type"] == "circle":
        r = fp["radius"]
        pts = [[0.0, -(r + side)]]
        for a in np.linspace(-math.pi / 2, math.pi / 2, 9)[1:-1]:
            pts.append([(r + margin) * math.cos(a), (r + side) * math.sin(a)])
        pts.append([0.0, r + side])
        return [[round(x, 3), round(y, 3)] for x, y in pts]
    poly = np.array(fp["polygon"], float)
    front = poly[:, 0].max() + margin
    xc = (poly[:, 0].max() + poly[:, 0].min()) / 2
    half = np.abs(poly[:, 1]).max() + side
    return [[round(front, 3), round(half, 3)], [round(xc, 3), round(half, 3)],
            [round(xc, 3), round(-half, 3)], [round(front, 3), round(-half, 3)]]

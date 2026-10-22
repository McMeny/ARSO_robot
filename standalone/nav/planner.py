"""Runtime C navigation stand-in for Nav2: inflated costmap, A*, footprint path check, pure pursuit, wander.

Only used by the standalone sim; the ROS runtimes use Nav2 with the same footprint and tolerances.
"""
from __future__ import annotations

import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

from eyesteer_core.control.diffdrive_kinematics import wrap
from eyesteer_core.control.footprint import Footprint, point_in_poly
from eyesteer_core.world import Grid, rasterize


class Costmap:
    def __init__(self, world, fp_dict: dict, inflation_radius: float, resolution: float = 0.1,
                 lethal_margin: float = 0.06, path_clearance: float = 0.05):
        self.world = world
        self.fp = Footprint(fp_dict)
        self.res = resolution
        self.static = rasterize(world, resolution)
        self.inflation = inflation_radius
        if fp_dict["type"] == "circle":
            self.lethal_r = fp_dict["radius"] + fp_dict.get("padding", 0.0)
        else:
            self.lethal_r = fp_dict.get("inscribed_radius", 0.36) + lethal_margin
        self.path_clearance = path_clearance
        self.dyn = np.zeros_like(self.static.occ, dtype=float)    # decaying obstacle layer from LiDAR
        self.extra = np.zeros_like(self.static.occ, dtype=bool)   # cells forced lethal (path check)
        self._update()

    def _update(self):
        occ = self.static.occ | (self.dyn > 0.5)
        dist = distance_transform_edt(~occ) * self.res
        self.dist = dist
        lethal = (dist < self.lethal_r) | self.extra
        cost = np.where(dist < self.inflation, np.exp(-3.0 * (dist - self.lethal_r)), 0.0)
        self.cost = np.where(lethal, np.inf, 1.0 + 20.0 * np.clip(cost, 0, 1))

    def mark_points(self, pts_world: np.ndarray):
        self.dyn *= 0.0
        for x, y in pts_world:
            r, c = self.static.world_to_cell(x, y)
            if self.static.in_bounds(r, c) and not self.static.occ[r, c]:
                self.dyn[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = 1.0
        self._update()

    def clear_dynamic(self):
        self.dyn[:] = 0
        self.extra[:] = False
        self._update()

    def free(self, x, y) -> bool:
        r, c = self.static.world_to_cell(x, y)
        return self.static.in_bounds(r, c) and np.isfinite(self.cost[r, c])


def astar(cm: Costmap, start, goal, max_expand: int = 200000):
    g = cm.static
    s = g.world_to_cell(*start)
    t = g.world_to_cell(*goal)
    if not (g.in_bounds(*s) and g.in_bounds(*t)) or not np.isfinite(cm.cost[t]):
        return None
    cost = cm.cost.copy()
    if not np.isfinite(cost[s]):   # start inside inflation (e.g. after a stop): allow leaving it
        cost[s] = 50.0
        rr, cc = np.ogrid[:cost.shape[0], :cost.shape[1]]
        near = ((rr - s[0]) ** 2 + (cc - s[1]) ** 2) <= (int(0.5 / cm.res)) ** 2
        cost = np.where(near & ~np.isfinite(cost) & ~g.occ, 50.0, cost)
    nbrs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    openq = [(0.0, s)]
    came = {s: None}
    gs = {s: 0.0}
    n = 0
    while openq and n < max_expand:
        _, cur = heapq.heappop(openq)
        n += 1
        if cur == t:
            break
        for dr, dc, w in nbrs:
            nb = (cur[0] + dr, cur[1] + dc)
            if not g.in_bounds(*nb) or not np.isfinite(cost[nb]):
                continue
            ng = gs[cur] + w * cost[nb]
            if ng < gs.get(nb, 1e18):
                gs[nb] = ng
                came[nb] = cur
                h = math.hypot(nb[0] - t[0], nb[1] - t[1])
                heapq.heappush(openq, (ng + h, nb))
    if t not in came:
        return None
    cells = []
    cur = t
    while cur is not None:
        cells.append(cur)
        cur = came[cur]
    cells.reverse()
    pts = np.array([g.cell_to_world(r, c) for r, c in cells])
    pts[0] = start[:2]
    pts[-1] = goal[:2]
    return smooth(pts)


def smooth(pts: np.ndarray, iters: int = 30) -> np.ndarray:
    if len(pts) < 3:
        return pts
    p = pts.copy()
    for _ in range(iters):
        p[1:-1] = p[1:-1] + 0.5 * (pts[1:-1] - p[1:-1]) + 0.25 * (p[:-2] + p[2:] - 2 * p[1:-1])
    # resample at ~10 cm
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    s = np.arange(0, d[-1], 0.1)
    return np.stack([np.interp(np.r_[s, d[-1]], d, p[:, 0]), np.interp(np.r_[s, d[-1]], d, p[:, 1])], 1)


def path_clearance_violations(cm: Costmap, path: np.ndarray, sdf_fn, min_clear: float = 0.04) -> list:
    """Indices where the footprint (oriented along the path) gets closer than min_clear to static obstacles."""
    bad = []
    for i in range(0, len(path) - 1, 2):
        yaw = math.atan2(path[i + 1, 1] - path[i, 1], path[i + 1, 0] - path[i, 0])
        if cm.fp.clearance(path[i, 0], path[i, 1], yaw, sdf_fn) < min_clear:
            bad.append(i)
    return bad


def plan(cm: Costmap, start, goal, sdf_fn, tries: int = 4):
    """A* + footprint verification; offending cells become lethal and we replan."""
    for _ in range(tries):
        path = astar(cm, start, goal)
        if path is None:
            return None
        bad = path_clearance_violations(cm, path, sdf_fn, getattr(cm, "path_clearance", 0.05))
        bad = [i for i in bad if np.hypot(*(path[i] - np.asarray(start[:2]))) > 0.3
               and np.hypot(*(path[i] - np.asarray(goal[:2]))) > 0.3]
        if not bad:
            return path
        for i in bad:
            r, c = cm.static.world_to_cell(*path[i])
            cm.extra[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = True
        cm._update()
    return astar(cm, start, goal)


class PurePursuit:
    """Path follower -> (v, w). Rotates in place for large heading errors; final yaw alignment at the goal."""

    def __init__(self, v_max: float, w_max: float, lookahead: float = 0.6, xy_tol: float = 0.12, yaw_tol: float = 0.12):
        self.v_max, self.w_max = v_max, w_max
        self.lookahead = lookahead
        self.xy_tol, self.yaw_tol = xy_tol, yaw_tol
        self.path = None
        self.goal_yaw = None
        self.idx = 0
        self.done = False

    def set_path(self, path, goal_yaw=None):
        self.path, self.goal_yaw, self.idx, self.done = path, goal_yaw, 0, False

    def __call__(self, x, y, yaw) -> tuple[float, float]:
        if self.path is None or self.done:
            return 0.0, 0.0
        p = self.path
        gx, gy = p[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.xy_tol:
            if self.goal_yaw is None:
                self.done = True
                return 0.0, 0.0
            e = wrap(self.goal_yaw - yaw)
            if abs(e) < self.yaw_tol:
                self.done = True
                return 0.0, 0.0
            return 0.0, float(np.clip(2.0 * e, -self.w_max * 0.6, self.w_max * 0.6))
        d = np.hypot(p[:, 0] - x, p[:, 1] - y)
        self.idx = max(self.idx, int(np.argmin(d[self.idx:self.idx + 30])) + self.idx)
        j = self.idx
        while j < len(p) - 1 and math.hypot(p[j, 0] - x, p[j, 1] - y) < self.lookahead:
            j += 1
        tx, ty = p[j]
        e = wrap(math.atan2(ty - y, tx - x) - yaw)
        if abs(e) > 0.9:
            return 0.0, float(np.clip(2.0 * e, -self.w_max, self.w_max))
        Ld = max(math.hypot(tx - x, ty - y), 1e-3)
        curv = 2 * math.sin(e) / Ld
        v = self.v_max * max(0.25, 1 - 0.8 * abs(e)) * min(1.0, dist_goal / 0.8 + 0.3)
        w = float(np.clip(v * curv, -self.w_max, self.w_max))
        return float(v), w


def poly_contains(poly, x, y) -> bool:
    return bool(point_in_poly(np.array([[x, y]]), np.asarray(poly, float))[0])


class Wander:
    """Field robot idle behaviour: random reachable goals >= 2 m away within the field polygon."""

    def __init__(self, field_polygon, rng, min_dist: float = 2.0):
        self.poly = np.asarray(field_polygon, float)
        self.rng = rng
        self.min_dist = min_dist

    def next_goal(self, cm: Costmap, x, y):
        lo, hi = self.poly.min(0), self.poly.max(0)
        for _ in range(200):
            gx, gy = self.rng.uniform(lo, hi)
            if math.hypot(gx - x, gy - y) < self.min_dist or not poly_contains(self.poly, gx, gy):
                continue
            r, c = cm.static.world_to_cell(gx, gy)
            if cm.static.in_bounds(r, c) and cm.dist[r, c] > cm.lethal_r + 0.25:
                return gx, gy, None
        return None

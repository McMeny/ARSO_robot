"""MuJoCo world wrapper: robot drive, encoders, LiDAR, moving people, MR-overlay markers.

Shared by the standalone sim (Runtime C) and the ROS mujoco_bridge (Runtime A/A2), so the
physics, the LiDAR model and the sign conventions are identical in both.
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from eyesteer_core import config
from eyesteer_core.control.diffdrive_kinematics import RateLimiter, twist_to_wheels
from eyesteer_core.world import load_world, static_sdf

ROOT = Path(__file__).resolve().parents[2]
LIDAR_GROUPS = np.array([1, 0, 1, 0, 0, 0], dtype=np.uint8)   # world + people; never robot/MR/user


def yaw_from_quat(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])


class Person:
    def __init__(self, spec: dict, body_id: int, mocap_id: int):
        self.name = spec["name"]
        self.radius = spec["radius"]
        self.speed = spec["speed"]
        self.path = [np.array(p, float) for p in spec["path"]]
        self.delay = spec.get("start_delay", 0.0)
        self.body_id, self.mocap_id = body_id, mocap_id
        self.pos = self.path[0].copy()
        self.idx = 1
        self.dir = 1
        self.enabled = spec.get("enabled", True)
        self.blocked = False

    def step(self, dt, t, robot_clear_fn):
        if not self.enabled or t < self.delay or len(self.path) < 2:
            return
        target = self.path[self.idx]
        d = target - self.pos
        dist = np.linalg.norm(d)
        step = self.speed * dt
        nxt = self.pos + (d / dist * min(step, dist) if dist > 1e-6 else 0)
        # people are polite: they pause rather than walk INTO the robot (or the walker's user);
        # moving away / across is always allowed, so a crossing never deadlocks with the collision monitor
        c_next = robot_clear_fn(nxt[0], nxt[1])
        self.blocked = c_next < self.radius + 0.15 and c_next < robot_clear_fn(self.pos[0], self.pos[1])
        if self.blocked:
            self.blocked_for = getattr(self, "blocked_for", 0.0) + dt
            if self.blocked_for > 2.0:      # give way: turn back along the path
                self.dir *= -1
                self.idx = max(0, min(len(self.path) - 1, self.idx + self.dir))
                self.blocked_for = 0.0
            return
        self.blocked_for = 0.0
        self.pos = nxt
        if dist <= step:
            if self.idx + self.dir >= len(self.path) or self.idx + self.dir < 0:
                self.dir *= -1   # ping-pong along the path
            self.idx += self.dir


class SimWorld:
    def __init__(self, world: str, robot: str, scene_path: str | None = None, seed: int = 0,
                 start_pose=None, lidar_noise: float = 0.01, lidar_beams: int = 360):
        self.world_name, self.robot_name = world, robot
        path = scene_path or str(ROOT / f"config/generated/mjcf/scene_{world}_{robot}.xml")
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        self.world = load_world(world)
        self.rcfg = config.robot(robot)
        self.geom = config.geometry(robot)
        self.rng = np.random.default_rng(seed)
        w = self.rcfg["wheels"]
        self.wr, self.ws = w["radius"], w["separation"]
        L = self.rcfg["limits"]
        self.limiter = RateLimiter(L["v_max"], L["w_max"], L["a_max"], L["alpha_max"])
        self.base_body = self.m.body("base_footprint").id
        self.jl = self.m.joint("left_wheel_joint")
        self.jr = self.m.joint("right_wheel_joint")
        self.act_l = self.m.actuator("left_wheel_motor").id
        self.act_r = self.m.actuator("right_wheel_motor").id
        self.laser_site = self.m.site("laser_frame").id
        self.lidar_noise = lidar_noise
        self.n_beams = lidar_beams
        self.beam_angles = np.linspace(-math.pi, math.pi, lidar_beams, endpoint=False)
        masks = self.rcfg["lidar"].get("masked_sectors_deg") or []
        self.beam_valid = np.ones(lidar_beams, bool)
        for lo, hi in masks:
            deg = np.degrees(self.beam_angles) % 360
            self.beam_valid &= ~((deg >= lo) & (deg <= hi))
        self.people = []
        for spec in self.world.people:
            bid = self.m.body(spec["name"]).id
            self.people.append(Person(spec, bid, self.m.body_mocapid[bid]))
        self.marker_mocap = {}
        for b in range(self.m.nbody):
            nm = self.m.body(b).name
            if nm.startswith("marker_"):
                self.marker_mocap[int(nm.split("_")[1])] = (self.m.body_mocapid[b],
                                                            self.m.geom(f"{nm}_panel").id,
                                                            self.m.geom(f"{nm}_ring").id)
        self.robot_geoms = {g for g in range(self.m.ngeom) if self.m.geom_group[g] == 1}
        self.floor = self.m.geom("floor").id
        self.extra_obstacles: list = []   # (x, y, r) spawned at runtime
        self.wheel_cmd = (0.0, 0.0)
        self.cmd = (0.0, 0.0)
        if start_pose is not None:
            self.teleport(*start_pose)
        mujoco.mj_forward(self.m, self.d)
        for _ in range(100):   # settle on the floor
            mujoco.mj_step(self.m, self.d)

    # ------------------------------------------------------------------ state
    @property
    def t(self) -> float:
        return float(self.d.time)

    def pose(self) -> tuple[float, float, float]:
        q = self.d.qpos
        return float(q[0]), float(q[1]), yaw_from_quat(q[3:7])

    def twist(self) -> tuple[float, float]:
        x, y, yaw = self.pose()
        vx, vy = self.d.qvel[0], self.d.qvel[1]
        return float(math.cos(yaw) * vx + math.sin(yaw) * vy), float(self.d.qvel[5])

    def wheel_angles(self) -> tuple[float, float]:
        return float(self.d.qpos[self.jl.qposadr[0]]), float(self.d.qpos[self.jr.qposadr[0]])

    def wheel_velocities(self) -> tuple[float, float]:
        return float(self.d.qvel[self.jl.dofadr[0]]), float(self.d.qvel[self.jr.dofadr[0]])

    def imu(self) -> dict:
        s = self.d.sensor
        return {"gyro": self.d.sensor("imu_gyro").data.copy(), "accel": self.d.sensor("imu_accel").data.copy(),
                "quat": self.d.sensor("imu_quat").data.copy()}

    def teleport(self, x, y, yaw):
        self.d.qpos[0:3] = [x, y, 0.002]
        self.d.qpos[3:7] = quat_from_yaw(yaw)
        self.d.qvel[:] = 0
        self.limiter.v = self.limiter.w = 0.0
        mujoco.mj_forward(self.m, self.d)

    # ------------------------------------------------------------------ actuation
    def set_twist(self, v: float, w: float, dt: float):
        """Body twist command -> rate-limited -> wheel velocity actuators (same limits as the real robot)."""
        v, w = self.limiter(v, w, dt)
        self.cmd = (v, w)
        self.set_wheels(*twist_to_wheels(v, w, self.wr, self.ws))

    def set_wheels(self, wl: float, wr: float):
        """Direct wheel angular velocity (rad/s); used by the HIL MCU emulator."""
        self.wheel_cmd = (wl, wr)
        self.d.ctrl[self.act_l] = wl
        self.d.ctrl[self.act_r] = wr

    def step(self, dt: float):
        n = max(1, int(round(dt / self.m.opt.timestep)))
        for p in self.people:
            p.step(dt, self.t, self.robot_clearance_to_point)
            self.d.mocap_pos[p.mocap_id][:2] = p.pos
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)

    # ------------------------------------------------------------------ sensing
    def lidar(self, max_range: float = 12.0) -> tuple[np.ndarray, np.ndarray]:
        """Returns (angles in laser_frame, ranges); inf where nothing hit or the sector is masked."""
        pnt = self.d.site_xpos[self.laser_site].copy()
        R = self.d.site_xmat[self.laser_site].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        a = self.beam_angles + yaw
        vec = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], 1).reshape(-1).copy()
        geomid = np.zeros(self.n_beams, np.int32)
        dist = np.zeros(self.n_beams, np.float64)
        _multi_ray(self.m, self.d, pnt, vec, LIDAR_GROUPS, geomid, dist, self.n_beams, max_range)
        r = np.where((dist < 0) | (dist > max_range), np.inf, dist)
        # runtime-spawned obstacles are modelled analytically (no model recompilation)
        for ox, oy, orad in self.extra_obstacles:
            r = np.minimum(r, _ray_circle(pnt[:2], a, ox, oy, orad))
        r = r + self.rng.normal(0, self.lidar_noise, r.shape) * np.isfinite(r)
        r[~self.beam_valid] = np.inf
        return self.beam_angles.copy(), r

    def lidar_points_base(self, angles, ranges) -> np.ndarray:
        """LiDAR hits in the base_footprint frame (for the collision monitor)."""
        L = self.rcfg["lidar"]
        ok = np.isfinite(ranges)
        a, r = angles[ok] + L["yaw"], ranges[ok]
        return np.stack([L["x"] + r * np.cos(a), L["y"] + r * np.sin(a)], 1)

    # ------------------------------------------------------------------ geometry / safety
    def static_sdf(self, px, py):
        d = static_sdf(self.world, px, py)
        for ox, oy, orad in self.extra_obstacles:
            d = np.minimum(d, np.hypot(np.asarray(px) - ox, np.asarray(py) - oy) - orad)
        return d

    def people_circles(self):
        return [(p.pos[0], p.pos[1], p.radius) for p in self.people if p.enabled]

    def robot_clearance_to_point(self, px, py) -> float:
        from eyesteer_core.control.footprint import Footprint, point_in_poly
        fp = getattr(self, "_fp", None) or Footprint(self.geom["footprint"])
        self._fp = fp
        x, y, yaw = self.pose()
        poly = fp.world(x, y, yaw)
        if point_in_poly(np.array([[px, py]]), poly)[0]:
            return -1.0
        # distance to polygon edges
        best = 1e9
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            ab = b - a
            tt = np.clip(np.dot([px - a[0], py - a[1]], ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
            q = a + tt * ab
            best = min(best, math.hypot(px - q[0], py - q[1]))
        return best

    def contacts(self) -> list[tuple[str, str]]:
        """Robot-vs-world contacts (wheels/casters on the floor excluded)."""
        out = []
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if self.floor in (g1, g2):
                continue
            r1, r2 = g1 in self.robot_geoms, g2 in self.robot_geoms
            if r1 != r2:
                out.append((self.m.geom(g1).name, self.m.geom(g2).name))
        return out

    # ------------------------------------------------------------------ MR overlay markers
    def style_marker(self, mid: int, panel_rgba, ring_rgba, face_point=None):
        if mid not in self.marker_mocap:
            return
        mocap, panel, ring = self.marker_mocap[mid]
        self.m.geom_rgba[panel] = panel_rgba
        self.m.geom_rgba[ring] = ring_rgba
        if face_point is not None:   # billboard towards the viewer (yaw only)
            p = self.d.mocap_pos[mocap]
            yaw = math.atan2(face_point[1] - p[1], face_point[0] - p[0])
            self.d.mocap_quat[mocap] = quat_from_yaw(yaw)

    def spawn_obstacle(self, x, y, r):
        self.extra_obstacles.append((x, y, r))


def _ray_circle(origin, angles, cx, cy, r):
    ox, oy = origin[0] - cx, origin[1] - cy
    dx, dy = np.cos(angles), np.sin(angles)
    b = ox * dx + oy * dy
    c = ox * ox + oy * oy - r * r
    disc = b * b - c
    t = -b - np.sqrt(np.maximum(disc, 0))
    return np.where((disc >= 0) & (t > 0), t, np.inf)


def _multi_ray(m, d, pnt, vec, groups, geomid, dist, n, cutoff):
    """mj_multiRay across MuJoCo versions (3.3+ added an optional normals argument)."""
    try:
        mujoco.mj_multiRay(m, d, pnt, vec, groups, 1, -1, geomid, dist, n, cutoff)
    except TypeError:
        mujoco.mj_multiRay(m, d, pnt, vec, groups, 1, -1, geomid, dist, None, n, cutoff)

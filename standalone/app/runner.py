"""Runtime C closed loop: gaze -> synthetic EEG -> eyesteer_core BCI -> FSM -> planner -> mux ->
collision monitor -> MuJoCo. Records metrics; optionally renders composite video frames.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from eyesteer_core import config
from eyesteer_core.bci.eeg_sim import SyntheticEEG
from eyesteer_core.bci.pipeline import BciPipeline
from eyesteer_core.control.admittance import Admittance, pullback_scale
from eyesteer_core.control.diffdrive_kinematics import EncoderOdometry, wrap
from eyesteer_core.control.footprint import CollisionMonitor, Footprint, RotationGuard
from eyesteer_core.control.mux_rules import DeadMan, Mux, Twist
from eyesteer_core.fsm.mode_manager import ENGAGED, ESTOP, IDLE, ModeManager
from eyesteer_core.world import load_markers, marker_goal
from standalone.nav.planner import Costmap, PurePursuit, Wander, plan
from standalone.sim.simworld import SimWorld

DT = 1.0 / 30.0


@dataclass
class Metrics:
    selections: list = field(default_factory=list)       # (t, id, correct, gaze_id)
    arrivals: list = field(default_factory=list)         # (t, id, time_to_goal)
    latencies: list = field(default_factory=list)
    path_length: float = 0.0
    min_clearance: float = 1e9
    collisions: int = 0
    contacts: int = 0
    cm_activations: int = 0
    mux_time: dict = field(default_factory=dict)
    states_seen: list = field(default_factory=list)
    odom_drift_m: float = 0.0
    odom_drift_per_m: float = 0.0
    lookaround_time: float = 0.0
    nav_failures: int = 0

    def false_selections(self):
        return [s for s in self.selections if not s[2]]


class ScenarioRunner:
    def __init__(self, scn: dict, hud=None, seed: int | None = None):
        self.scn = scn
        self.seed = scn.get("seed", 1) if seed is None else seed
        self.rng = np.random.default_rng(self.seed)
        self.robot = scn["robot"]
        self.rcfg = config.robot(self.robot)
        self.is_walker = self.robot == "walker"
        self.markers = load_markers(scn.get("markers") or ("supermarket_default" if scn["world"] == "supermarket" else "field_default"))
        self.mk = {m.id: m for m in self.markers}
        self.cancel_ids = [m.id for m in self.markers if m.cancel]
        start = scn.get("start_pose")
        self.sim = SimWorld(scn["world"], self.robot, seed=self.seed, start_pose=start)
        for p in self.sim.people:
            if p.name in scn.get("disable_people", []):
                p.enabled = False
                self.sim.d.mocap_pos[p.mocap_id][:2] = [99, 99]
        for name, delay in (scn.get("people_delay") or {}).items():
            for p in self.sim.people:
                if p.name == name:
                    p.delay = delay
        self.t0 = self.sim.t
        bcfg = config.bci()
        self.fs = bcfg["fs"]
        self.eeg = SyntheticEEG(fs=self.fs, snr=scn.get("snr", 1.0), seed=self.seed)
        self.bci = BciPipeline(self.markers, bcfg, source="synthetic")
        self.fsm = ModeManager(idle_mode=self.rcfg["idle_mode"])
        self.fsm.set_markers([m.id for m in self.markers], self.cancel_ids)
        self.mux = Mux()
        self.deadman = DeadMan(self.rcfg.get("deadman", {}).get("release_timeout_s", 0.3))
        a = self.rcfg.get("admittance", {})
        self.adm = Admittance(a.get("mass", 25), a.get("damping", 40), a.get("yaw_inertia", 4), a.get("yaw_damping", 8),
                              a.get("force_deadband", 4), self.rcfg.get("handles", {}).get("y", 0.25),
                              self.rcfg["limits"]["v_max"], self.rcfg["limits"]["w_max"])
        fp = config.geometry(self.robot)["footprint"]
        self.fp = Footprint(fp)
        self.cm = CollisionMonitor(fp)
        self.rot_guard = RotationGuard(self.fp)
        nv = self.rcfg["nav"]
        self.costmap = Costmap(self.sim.world, fp, nv["inflation_radius"], lethal_margin=nv.get("lethal_margin", 0.06),
                               path_clearance=nv.get("path_clearance", 0.05))
        nav_v = scn.get("nav_speed", 0.45 if not self.is_walker else 0.4)
        self.pursuit = PurePursuit(nav_v, self.rcfg["limits"]["w_max"] * 0.8, lookahead=nv.get("lookahead", 0.6))
        self.wander = Wander(self.sim.world.field_polygon, np.random.default_rng(self.seed + 100))
        self.wander_v = scn.get("wander_speed", 0.3)
        self.nav_goal = None           # (kind, marker_id or None, x, y, yaw, t_sent)
        self.blocked_since = None
        self.progress = (0.0, 1e9)
        self.goal_cache = {}
        x, y, yaw = self.sim.pose()
        self.odom = EncoderOdometry(self.sim.wr, self.sim.ws, x, y, yaw)
        self.odom.update(*self.sim.wheel_angles(), 0.0)
        self.gt_track, self.odom_track = [], []
        self.last_pose = (x, y)
        self.metrics = Metrics()
        self.events = sorted(scn.get("events", []), key=lambda e: e["t"])
        self.ev_i = 0
        self.active = []               # timed events in effect
        self.estop_pressed = False
        self.grip = True
        self.force = (0.0, 0.0)
        self.camera = scn.get("camera", "chase")
        self.caption = ""
        self.scan = (np.zeros(0), np.zeros(0))
        self.scan_pts_world = np.zeros((0, 2))
        self.cm_state = "OK"
        self.mux_source = "NONE"
        self.out_twist = Twist()
        self.eeg_frac = 0.0
        self.oz_hist = np.zeros(self.fs * 4)
        self.last_selection = None
        self.attended = None
        self.engagement = 0.0
        self.look_plan = []
        self.flat_until = -1.0
        self.hud = hud
        # manual overrides (interactive viewer / ROS keyboard): None = use the scenario
        self.manual_gaze = None        # marker id, or 0 for "looking at nothing"
        self.manual_teleop = None      # Twist
        self.manual_grip = None
        self.manual_force = (0.0, 0.0)
        self.stills = []
        self._prev_state = None
        self._pending_lat = {}
        self.metrics.states_seen.append(self.fsm.status().display)

    # ------------------------------------------------------------------ helpers
    @property
    def t(self) -> float:
        return self.sim.t - self.t0

    def clearance(self, x, y, yaw) -> float:
        return self.fp.clearance(x, y, yaw, self.sim.static_sdf)

    def goal_for(self, mid):
        if mid not in self.goal_cache:
            m = self.mk[mid]
            self.goal_cache[mid] = marker_goal(m, self.rcfg["nav"]["approach_distance"], self.clearance, 0.15)
        return self.goal_cache[mid]

    def _start_nav(self, kind, mid, gx, gy, gyaw):
        x, y, yaw = self.sim.pose()
        self.costmap.clear_dynamic()
        path = plan(self.costmap, (x, y), (gx, gy), self.sim.static_sdf)
        if path is None:
            self.nav_goal = None
            return False
        self.pursuit.v_max = self.wander_v if kind == "wander" else self.scn.get("nav_speed", 0.45 if not self.is_walker else 0.4)
        self.pursuit.set_path(path, gyaw)
        self.nav_goal = (kind, mid, gx, gy, gyaw, self.t)
        self.progress = (self.t, math.hypot(gx - x, gy - y))
        self.blocked_since = None
        return True

    def _handle(self, cmds):
        for c in cmds:
            if c[0] == "send_goal":
                gx, gy, gyaw = self.goal_for(c[1])
                if not self._start_nav("marker", c[1], gx, gy, gyaw):
                    self.metrics.nav_failures += 1
                    self._handle(self.fsm.on_nav_failed(self.t, "no path"))
            elif c[0] in ("cancel_goal", "stop"):
                self.nav_goal = None
                self.pursuit.set_path(None)
            elif c[0] == "idle":
                self.nav_goal = None
                self.pursuit.set_path(None)
            elif c[0] == "arrived":
                pass

    # ------------------------------------------------------------------ gaze model
    def _gaze(self):
        if self.manual_gaze is not None:
            return (None if self.manual_gaze == 0 else self.manual_gaze), 1.0
        target, eng = None, 0.3
        for e in self.active:
            if "gaze" in e or "glance" in e:
                target = e.get("gaze", e.get("glance"))
                eng = e.get("engagement", 1.0 if "gaze" in e else 0.0)
        for (a, b, mid, e) in self.look_plan:
            if a <= self.t < b:
                target, eng = mid, e
        return target, eng

    def _plan_lookaround(self, e):
        t, end = e["t"], e["t"] + e["dur"]
        ids = [m.id for m in self.markers]
        while t < end:
            t += self.rng.uniform(1.5, 4.0)
            g = self.rng.uniform(0.25, e.get("max_glance", 0.8))
            if t + g < end:
                self.look_plan.append((t, t + g, int(self.rng.choice(ids)), float(self.rng.uniform(-1.5, 1.5))))
            t += g
        self.metrics.lookaround_time += e["dur"]

    # ------------------------------------------------------------------ events
    def _events(self):
        while self.ev_i < len(self.events) and self.events[self.ev_i]["t"] <= self.t + 1e-9:
            e = dict(self.events[self.ev_i])
            self.ev_i += 1
            if "dur" in e and "look_around" not in e:
                e["until"] = e["t"] + e["dur"]
                self.active.append(e)
            if "gaze" in e:
                self._pending_lat[e["gaze"]] = e["t"]
            if "look_around" in e:
                self._plan_lookaround(e)
            if e.get("estop") is True:
                self.estop_pressed = True
                self._handle(self.fsm.on_estop(self.t, True))
            if e.get("estop_release"):
                self.estop_pressed = False
            if e.get("reset"):
                self.estop_pressed = False
                self._handle(self.fsm.reset(self.t))
            if "camera" in e:
                self.camera = e["camera"]
            if "caption" in e:
                self.caption = e["caption"]
            if "spawn_obstacle" in e:
                o = e["spawn_obstacle"]
                self.sim.spawn_obstacle(o["x"], o["y"], o.get("r", 0.25))
            if "link" in e:
                self._handle(self.fsm.on_link(self.t, e["link"], e.get("ok", False)))
            if "teleport" in e:
                self.sim.teleport(*e["teleport"])
                self._handle(self.fsm.on_localization(self.t, False))
            if "localization_ok" in e:
                self._handle(self.fsm.on_localization(self.t, True))
            if "bad_signal" in e:
                self.flat_until = e["t"] + e["bad_signal"]
        self.active = [e for e in self.active if e["until"] > self.t]
        self.grip = not any(e.get("grip") is False for e in self.active)
        fl = fr = 0.0
        for e in self.active:
            if "pull" in e:
                fl += e["pull"]
                fr += e["pull"]
            if "push" in e:
                p = e["push"]
                fl += p if not isinstance(p, list) else p[0]
                fr += p if not isinstance(p, list) else p[1]
        if self.manual_grip is not None:
            self.grip = self.manual_grip
        self.force = (fl + self.manual_force[0], fr + self.manual_force[1])

    # ------------------------------------------------------------------ main step
    def step(self):
        t = self.t
        self._events()
        # --- BCI
        target, eng = self._gaze()
        self.attended, self.engagement = target, eng
        freq = self.mk[target].frequency if target in self.mk else None
        self.eeg_frac += self.fs * DT
        n = int(self.eeg_frac)
        self.eeg_frac -= n
        x_eeg = self.eeg.generate(n, freq, eng)
        if t < self.flat_until:
            x_eeg[:, 4] = 0.0
        self.oz_hist = np.r_[self.oz_hist[n:], x_eeg[:, 4]]
        sels = self.bci.push(x_eeg, t)
        sig_ok = self.bci.status.signal_ok
        if self.is_walker and hasattr(self, "_sig_ok") and sig_ok != self._sig_ok and self.bci.status.t > 2.5:
            self._handle(self.fsm.on_link(t, "bci", sig_ok))
        self._sig_ok = sig_ok
        for s in sels:
            correct = s.marker_id == self._recent_gaze(t)
            self.metrics.selections.append((round(t, 2), s.marker_id, bool(correct), self._recent_gaze(t)))
            if s.marker_id in self._pending_lat:
                self.metrics.latencies.append(round(t - self._pending_lat.pop(s.marker_id), 3))
            self.last_selection = (t, s.marker_id)
            self._handle(self.fsm.on_selection(s.marker_id, t))
        self._handle(self.fsm.tick(t))

        # --- localization + odometry (the standalone uses ground truth as the AMCL estimate)
        x, y, yaw = self.sim.pose()
        self.odom.update(*self.sim.wheel_angles(), DT)
        self.metrics.path_length += math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y)
        if int(round(t / DT)) % 3 == 0:
            self.gt_track.append((x, y))
            self.odom_track.append((self.odom.x, self.odom.y))

        # --- LiDAR at 10 Hz
        if int(round(t / DT)) % 3 == 0:
            self.scan = self.sim.lidar()
            pts = self.sim.lidar_points_base(*self.scan)
            c, s_ = math.cos(yaw), math.sin(yaw)
            self.scan_pts_world = np.stack([x + c * pts[:, 0] - s_ * pts[:, 1], y + s_ * pts[:, 0] + c * pts[:, 1]], 1) if len(pts) else np.zeros((0, 2))
            self._cm_pts = pts
            self.rot_guard.add_scan(t, self.scan_pts_world)

        # --- idle behaviour + navigation
        st = self.fsm.state
        if st == IDLE and self.fsm.idle_mode == "WANDER" and (self.nav_goal is None or self.nav_goal[0] != "wander"):
            g = self.wander.next_goal(self.costmap, x, y)
            if g:
                self._start_nav("wander", None, g[0], g[1], None)
        nav = Twist()
        if self.nav_goal is not None and st in (ENGAGED, IDLE):
            v, w = self.pursuit(x, y, yaw)
            kind, mid, gx, gy, gyaw, ts = self.nav_goal
            d = math.hypot(gx - x, gy - y)
            if self.pursuit.done:
                if kind == "marker":
                    ok = d <= self.rcfg["nav"]["goal_xy_tolerance"] and (gyaw is None or abs(wrap(gyaw - yaw)) <= math.radians(self.rcfg["nav"]["goal_yaw_tolerance_deg"]))
                    self.nav_goal = None
                    if ok:
                        self.metrics.arrivals.append((round(t, 2), mid, round(t - ts, 2)))
                        self._handle(self.fsm.on_nav_succeeded(t))
                    else:
                        self._handle(self.fsm.on_nav_failed(t, "goal tolerance"))
                else:
                    self.nav_goal = None
            else:
                nav = Twist(v, w)
                # progress / blocked handling (Nav2 recovery stand-in): replan around LiDAR obstacles
                if d < self.progress[1] - 0.2:
                    self.progress = (t, d)
                elif t - self.progress[0] > 6.0:
                    self.costmap.mark_points(self.scan_pts_world)
                    path = plan(self.costmap, (x, y), (gx, gy), self.sim.static_sdf)
                    if path is not None:
                        self.pursuit.set_path(path, gyaw)
                    self.progress = (t, d)
                    if kind == "marker" and path is None:
                        self.nav_goal = None
                        self.metrics.nav_failures += 1
                        self._handle(self.fsm.on_nav_failed(t, "blocked"))
        # walker: pull-back scales navigation (the user always wins)
        if self.is_walker and st == ENGAGED:
            a = self.rcfg["admittance"]
            k = pullback_scale(*self.force, a["pullback_threshold"], a["pullback_stop"])
            nav = nav.scaled(k)
        self.mux.input("navigation", nav, t)
        for e in self.active:
            if "teleop" in e:
                self.mux.input("teleop", Twist(*e["teleop"]), t)
        if self.manual_teleop is not None:
            self.mux.input("teleop", self.manual_teleop, t)
        if self.is_walker and st == IDLE:
            v, w = self.adm.step(*self.force, DT)
            if self.adm.active(*self.force):
                self.mux.input("admittance", Twist(v, w), t)
        else:
            self.adm.reset()
        self.mux.lock("estop", self.estop_pressed or st == ESTOP)
        if self.is_walker:
            self.mux.lock("deadman", self.deadman.update(self.grip, t))
        out, src = self.mux.output(t)
        # collision monitor after the mux (applies to every source)
        scale, self.cm_state = self.cm(getattr(self, "_cm_pts", np.zeros((0, 2))))
        if out.v > 0:
            out = Twist(out.v * scale, out.w if scale > 0 else 0.0)   # STOP zeroes everything (as Nav2)
        # in-place rotation guard (sweeps the whole footprint incl. the walker's user)
        if self.fp.type == "polygon" and abs(out.v) < 0.05 and abs(out.w) > 0.05 and self.rot_guard.blocked(x, y, yaw):
            out = Twist(out.v, 0.0)
            self.cm_state = "STOP"
        # Nav2-style recovery: stopped by the collision monitor for > 4 s while navigating -> back up, replan
        if self.cm_state == "STOP" and src == "NAVIGATION" and self.nav_goal is not None:
            self._stop_since = getattr(self, "_stop_since", None) or t
            if t - self._stop_since > 4.0:
                self._backup_until = t + 2.0
                self._stop_since = None
        else:
            self._stop_since = None
        if t < getattr(self, "_backup_until", -1) and src == "NAVIGATION":
            out = Twist(-0.12, 0.0)
            if t + DT >= self._backup_until and self.nav_goal is not None:
                self.costmap.mark_points(self.scan_pts_world)
                kind, mid, gx, gy, gyaw, ts = self.nav_goal
                path = plan(self.costmap, (x, y), (gx, gy), self.sim.static_sdf)
                if path is not None:
                    self.pursuit.set_path(path, gyaw)
        self.out_twist, self.mux_source = out, src
        self.sim.set_twist(out.v, out.w, DT)
        self.sim.step(DT)

        # --- metrics
        mt = self.metrics
        mt.mux_time[src] = mt.mux_time.get(src, 0.0) + DT
        clear = self.fp.clearance(*self.sim.pose(), self.sim.static_sdf, self.sim.people_circles())
        if clear < 0 and getattr(self, "_was_clear", True):
            mt.collisions += 1
        self._was_clear = clear >= 0
        mt.min_clearance = min(mt.min_clearance, clear)
        if self.sim.contacts():
            mt.contacts += 1
        mt.cm_activations = self.cm.activations
        disp = self.fsm.status().display
        if disp != self._prev_state:
            key = disp.split(" ")[0]
            if not mt.states_seen or mt.states_seen[-1] != key:
                mt.states_seen.append(key)
            self._prev_state = disp

    def _recent_gaze(self, t):
        """Ground truth: the marker the user attended most in the last 3 s (None = no intent)."""
        best, best_t = None, 0.0
        for mid in self.mk:
            dur = 0.0
            for e in self.events:
                g = e.get("gaze")
                if g == mid:
                    a, b = e["t"], e["t"] + e["dur"]
                    dur += max(0.0, min(b, t) - max(a, t - 3.0))
            if dur > best_t:
                best, best_t = mid, dur
        return best if best_t >= 0.75 else None

    def finish(self) -> dict:
        x, y, _ = self.sim.pose()
        mt = self.metrics
        mt.odom_drift_m = round(math.hypot(self.odom.x - x, self.odom.y - y), 4)
        mt.odom_drift_per_m = round(mt.odom_drift_m / max(mt.path_length, 1e-6), 5)
        total = sum(mt.mux_time.values()) or 1.0
        false = mt.false_selections()
        minutes = max(mt.lookaround_time, 1e-9) / 60.0
        return {
            "scenario": self.scn["name"], "robot": self.robot, "world": self.scn["world"], "duration_s": round(self.t, 2),
            "eeg": "SIMULATED", "selections": [{"t": s[0], "id": s[1], "label": self.mk[s[1]].label, "correct": s[2]} for s in mt.selections],
            "correct_selections": sum(1 for s in mt.selections if s[2]), "false_selections": len(false),
            "false_per_min_lookaround": round(len(false) / minutes, 3) if mt.lookaround_time else None,
            "detection_latency_s": mt.latencies, "arrivals": [{"t": a[0], "id": a[1], "label": self.mk[a[1]].label, "time_to_goal_s": a[2]} for a in mt.arrivals],
            "path_length_m": round(mt.path_length, 2), "min_clearance_m": round(mt.min_clearance, 3),
            "collisions": mt.collisions, "contact_frames": mt.contacts, "collision_monitor_activations": mt.cm_activations,
            "mux_share": {k: round(v / total, 3) for k, v in sorted(mt.mux_time.items())},
            "states_seen": mt.states_seen, "final_state": self.fsm.status().display.split(" ")[0],
            "odom_drift_m": mt.odom_drift_m, "odom_drift_per_m": mt.odom_drift_per_m, "nav_failures": mt.nav_failures,
        }

    def snapshot(self) -> dict:
        """Everything the HUD needs for one frame."""
        st = self.fsm.status()
        return {"t": self.t, "fsm": st, "mux": self.mux_source, "cm": self.cm_state, "grip": self.grip,
                "force": self.force, "pose": self.sim.pose(), "path": self.pursuit.path if self.nav_goal else None,
                "goal": self.nav_goal, "scan": self.scan_pts_world, "bci": self.bci.status, "oz": self.oz_hist,
                "attended": self.attended, "caption": self.caption, "gt_track": self.gt_track,
                "odom_track": self.odom_track, "last_selection": self.last_selection, "estop": self.estop_pressed,
                "twist": self.out_twist, "markers": self.markers}

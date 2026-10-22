"""scenario_player: runs config/scenarios/*.yaml against the ROS sim stack (same files as Runtime C).

Publishes the scenario's human inputs as the real system would receive them (synthetic gaze -> Muse
LSL emulator, teleop, e-stop button, grip, handle forces, obstacles, faults), records what the stack
does (/robot/status, /bci/selection, /mux/source, collision monitor, ground truth) and checks the SAME
expectations as standalone/app/run_scenario.py. Writes outputs/metrics/ros_<scenario>.json.
The scenario clock starts when the health gates unlock motion (gated startup).

Per-scenario `ros:` overrides (Nav2 drives differently from the Runtime C planner):
  ros: {time_scale: 1.5, duration: 140}   # event times after t=0 are stretched by time_scale
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

from eyesteer_behavior import _root

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose2D, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Float32MultiArray, Int32, String

from eyesteer_core import config
from eyesteer_core.control.footprint import Footprint
from eyesteer_core.world import load_world, static_sdf

from eyesteer_interfaces.msg import RobotStatus, Selection, SystemHealth

ROOT = _root.ROOT


class ScenarioPlayer(Node):
    def __init__(self):
        super().__init__("scenario_player")
        name = self.declare_parameter("scenario", "field_basic").value
        self.out_dir = Path(self.declare_parameter("out", str(ROOT / "outputs/metrics")).value)
        self.wait_unlock = self.declare_parameter("wait_for_unlock", True).value
        self.prefix = self.declare_parameter("prefix", "ros").value     # ros (mujoco backend) | hil
        self.scn = config.scenario(name)
        ros = self.scn.get("ros", {})
        self.k = float(ros.get("time_scale", 1.0))
        self.duration = float(ros.get("duration", self.scn["duration"] * self.k))
        self.events = sorted(self.scn.get("events", []), key=lambda e: e["t"])
        for e in self.events:
            e["t"] = e["t"] * self.k
        self.robot = self.scn["robot"]
        self.world = load_world(self.scn["world"])
        self.fp = Footprint(config.geometry(self.robot)["footprint"])
        self.rng = np.random.default_rng(self.scn.get("seed", 1))
        self.look_plan = []
        self.lookaround_time = 0.0
        self.t0 = None
        self.ev_i = 0
        self.active = []
        self.status = None
        self.unlocked = False
        self.sel, self.arrivals, self.states = [], [], []
        self.mux_time, self.cm_state, self.cm_act = {}, "OK", 0
        self.min_clear, self.collisions, self.was_clear = 1e9, 0, True
        self.path_len, self.last_xy, self.pose = 0.0, None, None
        self.people = []
        self.prev_goal = None
        self.goal_sent_t = {}
        self.latencies, self.lat_done = [], set()
        self.pub_gaze = self.create_publisher(Int32, "/bci/gaze", 10)
        self.pub_eng = self.create_publisher(Float32, "/bci/engagement", 10)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_estop_btn = self.create_publisher(Bool, "/sim/estop_button", 10)
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self.pub_reset = self.create_publisher(Empty, "/system/reset", 10)
        self.pub_grip = self.create_publisher(Bool, "/sim/grip_cmd", 10)
        self.pub_force = self.create_publisher(Float32MultiArray, "/sim/handle_force_cmd", 10)
        self.pub_obst = self.create_publisher(Point, "/sim/spawn_obstacle", 10)
        self.pub_tele = self.create_publisher(Pose2D, "/sim/teleport", 10)
        self.pub_fault = self.create_publisher(String, "/sim/eeg_fault", 10)
        self.pub_people = self.create_publisher(String, "/sim/people_cmd", 10)
        self.create_subscription(RobotStatus, "/robot/status", self.on_status, 10)
        self.create_subscription(Selection, "/bci/selection", self.on_sel, 10)
        self.create_subscription(String, "/mux/source", self.on_mux, 10)
        self.create_subscription(String, "/collision_monitor/state", self.on_cm, 10)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_gt, 10)
        self.create_subscription(PoseArray, "/sim/people", lambda m: setattr(self, "people", [(p.position.x, p.position.y, p.position.z) for p in m.poses]), 10)
        self.create_subscription(SystemHealth, "/system/health", lambda m: setattr(self, "unlocked", m.motion_unlocked), 10)
        self.last_mux_t = None
        self.done = False
        self.result = None
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"scenario {name}: duration {self.duration:.0f} s (time_scale {self.k}); waiting for motion unlock")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def t(self) -> float:
        return self.now() - self.t0 if self.t0 is not None else -1.0

    # ------------------------------------------------------------------ recording
    def on_status(self, m: RobotStatus):
        self.status = m
        if self.t0 is None:
            return
        key = m.display.split(" ")[0]
        if not self.states or self.states[-1] != key:
            self.states.append(key)
        if m.state == "ARRIVED" and self.prev_goal != ("ARRIVED", m.goal_id):
            sent = self.goal_sent_t.pop(m.goal_id, None)
            self.arrivals.append({"t": round(self.t, 2), "id": m.goal_id, "label": m.goal_label,
                                  "time_to_goal_s": round(self.t - sent, 2) if sent is not None else None})
        if m.state == "ENGAGED" and m.goal_id not in self.goal_sent_t:
            self.goal_sent_t[m.goal_id] = self.t
        self.prev_goal = (m.state, m.goal_id) if m.state == "ARRIVED" else None

    def on_sel(self, m: Selection):
        if self.t0 is None:
            return
        gt = self.recent_gaze(self.t)
        onset = [e["t"] for e in self.events if e.get("gaze") == m.marker_id and e["t"] <= self.t]
        if onset and m.marker_id not in self.lat_done:
            self.lat_done.add(m.marker_id)
            self.latencies.append(round(self.t - onset[-1], 3))
        self.sel.append({"t": round(self.t, 2), "id": m.marker_id, "label": m.label, "correct": m.marker_id == gt})

    def on_mux(self, m: String):
        if self.t0 is None:
            return
        t = self.t
        if self.last_mux_t is not None:
            self.mux_time[m.data] = self.mux_time.get(m.data, 0.0) + (t - self.last_mux_t)
        self.last_mux_t = t

    def on_cm(self, m: String):
        if m.data != "OK" and self.cm_state == "OK" and self.t0 is not None:
            self.cm_act += 1
        self.cm_state = m.data

    def on_gt(self, m: Odometry):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        self.pose = (x, y, yaw)
        if self.t0 is None:
            return
        if self.last_xy is not None:
            self.path_len += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        c = self.fp.clearance(x, y, yaw, lambda px, py: static_sdf(self.world, px, py), self.people)
        if c < 0 and self.was_clear:
            self.collisions += 1
            cs = self.fp.clearance(x, y, yaw, lambda px, py: static_sdf(self.world, px, py))
            near = min(self.people, key=lambda p: math.hypot(p[0] - x, p[1] - y)) if self.people else None
            self.get_logger().error(f"COLLISION t={self.t:.1f} pose=({x:.2f},{y:.2f},{yaw:.2f}) static_clear={cs:.3f} "
                                    f"nearest_person={near} state={self.status.display if self.status else '?'}")
        self.was_clear = c >= 0
        self.min_clear = min(self.min_clear, c)

    def recent_gaze(self, t):
        best, best_t = None, 0.0
        for e in self.events:
            g = e.get("gaze")
            if g is None:
                continue
            a, b = e["t"], e["t"] + e["dur"]
            d = max(0.0, min(b, t) - max(a, t - 3.0 * self.k))
            if d > best_t:
                best, best_t = g, d
        return best if best_t >= 0.75 else None

    # ------------------------------------------------------------------ playback
    def plan_lookaround(self, e):
        t, end = e["t"], e["t"] + e["dur"] * self.k
        ids = [1, 2, 3, 4, 5, 6]
        while t < end:
            t += self.rng.uniform(1.5, 4.0)
            g = self.rng.uniform(0.25, e.get("max_glance", 0.8))
            if t + g < end:
                self.look_plan.append((t, t + g, int(self.rng.choice(ids)), float(self.rng.uniform(-1.5, 1.5))))
            t += g
        self.lookaround_time += e["dur"] * self.k

    def tick(self):
        if self.done:
            return
        if self.t0 is None:
            if self.unlocked or not self.wait_unlock:
                self.t0 = self.now()
                self.get_logger().info("motion unlocked: scenario clock started")
                cmd = json.dumps({"disable": self.scn.get("disable_people", []), "delay": self.scn.get("people_delay", {})})
                for _ in range(3):   # same people schedule as Runtime C, relative to the scenario clock
                    self.pub_people.publish(String(data=cmd))
            return
        t = self.t
        while self.ev_i < len(self.events) and self.events[self.ev_i]["t"] <= t:
            e = dict(self.events[self.ev_i])
            self.ev_i += 1
            if "dur" in e and "look_around" not in e:
                dur = e["dur"] if ("gaze" in e or "glance" in e) else e["dur"] * self.k   # gaze timing is physiological
                e["until"] = e["t"] + dur
                self.active.append(e)
            if "look_around" in e:
                self.plan_lookaround(e)
            if e.get("estop") is True:
                self.pub_estop_btn.publish(Bool(data=True))
                self.pub_estop.publish(Bool(data=True))
            if e.get("estop_release"):
                self.pub_estop_btn.publish(Bool(data=False))
            if e.get("reset"):
                self.pub_estop_btn.publish(Bool(data=False))
                self.pub_reset.publish(Empty())
            if "spawn_obstacle" in e:
                o = e["spawn_obstacle"]
                self.pub_obst.publish(Point(x=float(o["x"]), y=float(o["y"]), z=float(o.get("r", 0.25))))
            if "teleport" in e:
                x, y, th = e["teleport"]
                self.pub_tele.publish(Pose2D(x=float(x), y=float(y), theta=float(th)))
            if "link" in e:
                self.pub_fault.publish(String(data="ok" if e.get("ok") else "drop"))
            if "bad_signal" in e:
                e2 = {"until": e["t"] + e["bad_signal"], "flat": True}
                self.active.append(e2)
        self.active = [e for e in self.active if e["until"] > t]
        # continuous inputs
        target, eng = 0, 0.3
        for e in self.active:
            if "gaze" in e or "glance" in e:
                target = e.get("gaze", e.get("glance"))
                eng = e.get("engagement", 1.0 if "gaze" in e else 0.0)
        for a, b, mid, en in self.look_plan:
            if a <= t < b:
                target, eng = mid, en
        self.pub_gaze.publish(Int32(data=int(target)))
        self.pub_eng.publish(Float32(data=float(eng)))
        if any(e.get("flat") for e in self.active):
            self.pub_fault.publish(String(data="flat"))
        fl = fr = 0.0
        grip = True
        for e in self.active:
            if "teleop" in e:
                tw = Twist()
                tw.linear.x, tw.angular.z = float(e["teleop"][0]), float(e["teleop"][1])
                self.pub_teleop.publish(tw)
            if "pull" in e:
                fl += e["pull"]
                fr += e["pull"]
            if "push" in e:
                p = e["push"]
                fl += p if not isinstance(p, list) else p[0]
                fr += p if not isinstance(p, list) else p[1]
            if e.get("grip") is False:
                grip = False
        self.pub_grip.publish(Bool(data=grip))
        self.pub_force.publish(Float32MultiArray(data=[float(fl), float(fr)]))
        if t >= self.duration:
            self.finish()

    def finish(self):
        import sys
        sys.path.insert(0, str(ROOT))
        from standalone.app.run_scenario import check
        total = sum(self.mux_time.values()) or 1.0
        false = [s for s in self.sel if not s["correct"]]
        st = self.status
        res = {"scenario": self.scn["name"], "runtime": "ros_hil" if self.prefix == "hil" else "ros_sim", "robot": self.robot, "world": self.scn["world"],
               "duration_s": round(self.t, 1), "eeg": "SIMULATED", "selections": self.sel,
               "correct_selections": len(self.sel) - len(false), "false_selections": len(false),
               "false_per_min_lookaround": round(len(false) / (self.lookaround_time / 60), 3) if self.lookaround_time else None,
               "arrivals": self.arrivals, "detection_latency_s": self.latencies, "path_length_m": round(self.path_len, 2),
               "min_clearance_m": round(self.min_clear, 3), "collisions": self.collisions,
               "collision_monitor_activations": self.cm_act,
               "mux_share": {k: round(v / total, 3) for k, v in sorted(self.mux_time.items())},
               "states_seen": self.states, "final_state": st.display.split(" ")[0] if st else "?"}
        scn = dict(self.scn)
        scn["expect"] = {k: v for k, v in scn.get("expect", {}).items() if k not in self.scn.get("ros", {}).get("skip_expect", [])}
        res["expectation_failures"] = check(scn, res)
        res["passed"] = not res["expectation_failures"]
        self.out_dir.mkdir(parents=True, exist_ok=True)
        (self.out_dir / f"{self.prefix}_{self.scn['name']}.json").write_text(json.dumps(res, indent=2))
        self.get_logger().info(f"[ROS {self.scn.get('id', '')} {self.scn['name']}] {'PASS' if res['passed'] else 'FAIL'} "
                               f"selections={[s['label'] for s in self.sel]} visited={[a['label'] for a in self.arrivals]} "
                               f"states={self.states} collisions={self.collisions} {res['expectation_failures']}")
        self.result = res
        self.done = True


def main():
    rclpy.init()
    n = ScenarioPlayer()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    code = 0 if (n.result and n.result["passed"]) else 1
    time.sleep(0.2)
    rclpy.try_shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()

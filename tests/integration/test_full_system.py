"""Full-system golden path (Section 24.3) in backend:=mujoco_hil, 5 consecutive runs per robot.

Everything real except photons, scalp and motors: MuJoCo (physics + LiDAR) <- MCU emulator (the firmware's C
core on a pty) <- mcu_bridge <- ros2_control + diffdrive_arduino <- safety <- collision monitor <- twist_mux <- Nav2
(AMCL on the saved map) <- mode_manager <- ssvep_node <- LSL <- Muse emulator <- synthetic gaze <- the headless
headset client (the real WebXR app over the real rosbridge).

Run:  python -m pytest -q tests/integration/test_full_system.py      (results: outputs/metrics/golden_<robot>.json)
"""
from __future__ import annotations

import json
import math
import statistics
import time
from pathlib import Path

import numpy as np
import pytest

from harness import ROOT, headset_client, ros_available, sim_stack, wait_state

from eyesteer_core import config
from eyesteer_core.control.footprint import Footprint
from eyesteer_core.world import load_markers, load_world, static_sdf

pytestmark = pytest.mark.skipif(not ros_available(), reason="ROS 2 workspace not available")
RUNS = 5
BUDGET = {   # Section 24.7
    "gaze onset -> selection (s)": (1.25, 3.0),
    "selection -> NavigateToPose accepted (ms)": (0, 200),
    "goal accepted -> robot moving (ms)": (0, 1000),
    "cmd_vel -> wheel response, HIL (ms)": (0, 100),
    "robot state change -> visible in headset (ms)": (0, 300),
}
RESULTS = {"field_robot": [], "walker": []}


class Tracker:
    """Samples the ground-truth pose + people via the page and checks clearance (collisions must be 0)."""

    def __init__(self, page, robot, world):
        self.page, self.fp = page, Footprint(config.geometry(robot)["footprint"])
        self.world = load_world(world)
        self.min_clear = 1e9
        self.collisions = 0
        self.was_clear = True
        self.speeds = []

    def sample(self):
        s = self.page.evaluate("(() => { const s = window.eyesteer.state(); return {pose: s.pose, people: s.people, status: s.status}; })()")
        p = s["pose"]
        if not p:
            return s
        q = p["orientation"]
        yaw = math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))
        x, y = p["position"]["x"], p["position"]["y"]
        c = self.fp.clearance(x, y, yaw, lambda px, py: static_sdf(self.world, px, py), [tuple(v) for v in s["people"]])
        self.min_clear = min(self.min_clear, c)
        if c < 0 and self.was_clear:
            self.collisions += 1
        self.was_clear = c >= 0
        s["xy"] = (x, y, time.time())
        self.speeds.append(s["xy"])
        return s

    def speed(self, window=0.6):
        pts = [p for p in self.speeds if time.time() - p[2] <= window]
        if len(pts) < 2:
            return 0.0
        return math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) / max(pts[-1][2] - pts[0][2], 1e-3)


def run_until(page, tr, pred_js, timeout, every=0.25):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = tr.sample()
        ok = page.evaluate(f"(() => {{ const s = window.eyesteer.state(); return !!({pred_js}); }})()")
        if ok:
            return s
        time.sleep(every)
    st = page.evaluate("window.eyesteer.state().status")
    raise TimeoutError(f"timeout: {pred_js} (status {st and st['display']}, queue {st and st['queue_labels']})")


def latencies(page):
    st = page.evaluate("window.eyesteer.state()")
    lat = st["latency"]
    off = st["clockOffset"] or 0.0          # robot wall clock - page clock (NTP-style ping)
    out = {k: [] for k in BUDGET}
    for e in lat:
        if e["event"] == "goal_accepted":
            out["selection -> NavigateToPose accepted (ms)"].append(e["dt_ms"])
        elif e["event"] == "moving":
            out["goal accepted -> robot moving (ms)"].append(e["dt_ms"])
        elif e["event"] == "wheel_response" and e.get("backend") == "mujoco_hil":
            out["cmd_vel -> wheel response, HIL (ms)"].append(e["dt_ms"])
        elif e["event"] == "state_visible":   # change -> shown in the page -> ack, on the robot's clock (upper bound)
            out["robot state change -> visible in headset (ms)"].append(e["dt_ms"])
    for sel in st["selections"]:
        aims = [a for a in st["aimLog"] if a["id"] == sel["id"] and a["t"] <= sel["t"]]
        if aims:
            out["gaze onset -> selection (s)"].append((sel["t"] - aims[-1]["t"]) / 1000)
    return out


def golden_field(page, tr):
    mk = load_markers("field_default")
    # 2. calibration (two decal touches; synthetic points from a known transform), residual < 5 cm
    r = page.evaluate("window.eyesteer.calibrateSim({yaw: 0.6, tx: 1.5, ty: -0.8, floorHeight: 0.0})")
    assert r["success"], r["message"]
    # 3. place 5 markers + CANCEL from the headset (headset frame -> anchor_bridge -> map)
    page.evaluate("(l) => window.eyesteer.placeHeadsetFrame(l)", [m.to_dict() for m in mk])
    st = wait_state(page, "s.markers.length === 6 && s.markers.find(m => m.label === 'C')", 20)
    c = next(m for m in st["markers"] if m["label"] == "C")
    assert abs(c["x"] - 4.5) < 0.01 and abs(c["y"] + 4.5) < 0.01        # round trip through the calibration
    # 4. gaze at C -> SSVEP over LSL -> selection -> Nav2 -> ... -> MuJoCo
    page.evaluate("window.eyesteer.aim(3)")
    run_until(page, tr, "s.status && s.status.state === 'ENGAGED' && s.status.goal_label === 'C'", 15)
    page.evaluate("window.eyesteer.aim(0)")
    # 5. during navigation: gaze at A -> queued, visible in the headset
    time.sleep(3)
    page.evaluate("window.eyesteer.aim(1)")
    run_until(page, tr, "s.status && s.status.queue_labels.includes('A')", 15)
    page.evaluate("window.eyesteer.aim(0)")
    # 6. a glance shorter than the dwell -> no selection
    n_sel = len(page.evaluate("window.eyesteer.state().selections"))
    page.evaluate("window.eyesteer.aim(5)")
    time.sleep(0.6)
    page.evaluate("window.eyesteer.aim(0)")
    time.sleep(4.0)
    sels = page.evaluate("window.eyesteer.state().selections")
    assert all(s["label"] != "E" for s in sels[n_sel:]), "a short glance fired a selection (Midas touch)"
    run_until(page, tr, "s.status && s.status.state === 'ARRIVED' && s.status.goal_label === 'C'", 120)
    run_until(page, tr, "s.status && s.status.state === 'ENGAGED' && s.status.goal_label === 'A'", 15)
    # CANCEL -> back to idle (WANDER), queue cleared
    time.sleep(4)
    page.evaluate("window.eyesteer.aim(6)")
    run_until(page, tr, "s.status && s.status.display === 'WANDER' && s.status.queue.length === 0", 15)
    page.evaluate("window.eyesteer.aim(0)")


def golden_walker(page, tr):
    mk = load_markers("supermarket_default")
    r = page.evaluate("window.eyesteer.calibrateSim({yaw: -0.3, tx: 0.5, ty: 0.2, floorHeight: 0.0})")
    assert r["success"], r["message"]
    page.evaluate("(l) => window.eyesteer.placeHeadsetFrame(l)", [m.to_dict() for m in mk])
    wait_state(page, "s.markers.length === 6 && s.markers.find(m => m.label === 'Beans')", 20)
    st = tr.sample()
    assert st["status"]["display"] == "ASSIST"
    time.sleep(2)
    assert tr.speed(1.5) < 0.02, "the walker must not move on its own in ASSIST"
    page.evaluate("window.eyesteer.aim(1)")
    run_until(page, tr, "s.status && s.status.state === 'ENGAGED' && s.status.goal_label === 'Beans'", 15)
    page.evaluate("window.eyesteer.aim(0)")
    run_until(page, tr, "true", 8)                        # get going
    v_before = max(tr.speed(1.0), 0.05)
    # pull back on the handles: navigation scaled down, the user wins
    page.evaluate("window.eyesteer.pub('/sim/handle_force_cmd', 'std_msgs/Float32MultiArray', {data: [-11, -11]})")
    run_until(page, tr, "true", 2.0)
    v_pull = tr.speed(0.8)
    page.evaluate("window.eyesteer.pub('/sim/handle_force_cmd', 'std_msgs/Float32MultiArray', {data: [0, 0]})")
    assert v_pull < 0.8 * v_before + 0.02, f"pull-back did not slow the walker ({v_before:.2f} -> {v_pull:.2f})"
    # dead-man: grip released -> stop within 0.3 s (+ deceleration), locked
    for _ in range(4):
        page.evaluate("window.eyesteer.pub('/sim/grip_cmd', 'std_msgs/Bool', {data: false})")
        time.sleep(0.25)
        tr.sample()
    run_until(page, tr, "s.status && s.status.deadman_locked", 3)
    time.sleep(0.8)
    tr.sample()
    assert tr.speed(0.5) < 0.05, "walker still moving with the grip released"
    for _ in range(3):
        page.evaluate("window.eyesteer.pub('/sim/grip_cmd', 'std_msgs/Bool', {data: true})")
        time.sleep(0.2)
    run_until(page, tr, "s.status && !s.status.deadman_locked", 5)
    run_until(page, tr, "s.status && s.status.state === 'ARRIVED' && s.status.goal_label === 'Beans'", 180)
    run_until(page, tr, "s.status && s.status.display === 'ASSIST'", 10)
    time.sleep(2)
    tr.sample()
    assert tr.speed(1.5) < 0.02, "after arrival the walker must stay put (ASSIST, never WANDER)"


@pytest.mark.parametrize("robot", ["field_robot", "walker"])
@pytest.mark.parametrize("run", range(RUNS))
def test_golden_path(robot, run):
    world = "field" if robot == "field_robot" else "supermarket"
    with sim_stack(world, robot, "mujoco_hil", require_headset=True, extra="record:=true",
                   log_name=f"golden_{robot}_{run}.log") as (proc, log):
        with headset_client() as (page, logs):
            tr = Tracker(page, robot, world)
            # 1. all health gates green (incl. MCU link through the emulator, headset, Muse/LSL)
            wait_state(page, "s.connected && s.markers.length >= 6", 180)
            page.evaluate("window.eyesteer.consent()")
            st = wait_state(page, "s.health && s.health.motion_unlocked && s.health.all_required_ok", 180)
            red = [c["name"] for c in st["health"]["components"] if c["level"] != 0 and c["required"]]
            assert not red and st["health"]["mode"] == "hil"
            (golden_field if robot == "field_robot" else golden_walker)(page, tr)
            assert tr.collisions == 0, f"collision (min clearance {tr.min_clear:.3f} m)"
            lat = latencies(page)
            rtt = page.evaluate("window.eyesteer.state().clockRtt")
            page.screenshot(path=str(ROOT / f"outputs/stills/golden_{robot}.png"))
        # 8. recordings exist (rosbag + XDF of the LSL stream)
        bag = ROOT / "outputs/logs/rosbag"
        xdf = list((ROOT / "outputs/logs").glob(f"eeg_{robot}_{world}*.xdf"))
        assert xdf and xdf[0].stat().st_size > 10000, "no XDF recording"
        assert bag.exists() and any(bag.iterdir()), "no rosbag recording"
    for k, (lo, hi) in BUDGET.items():
        assert lat[k], f"no latency samples for {k}"
        med = statistics.median(lat[k])
        assert lo <= med <= hi, f"latency budget violated: {k} median {med:.3g} not in [{lo:.3g}, {hi}]"
    RESULTS[robot].append({k: statistics.median(v) for k, v in lat.items() if v})
    out = ROOT / "outputs/metrics" / f"golden_{robot}.json"
    out.write_text(json.dumps({"robot": robot, "backend": "mujoco_hil", "runs": RUNS, "passes": len(RESULTS[robot]),
                               "min_clearance_m": round(tr.min_clear, 3),
                               "latency": {k: {"measured": round(statistics.median([r[k] for r in RESULTS[robot] if k in r]), 3),
                                               "target": f"{lo}-{hi}"} for k, (lo, hi) in BUDGET.items()}}, indent=2))

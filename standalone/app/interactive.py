#!/usr/bin/env python3
"""Interactive Runtime C: MuJoCo viewer + the same key bindings as the ROS mujoco_bridge (Section 10).

  1..5   synthetic gaze at marker slot      C      gaze at CANCEL        0   look at nothing
  arrows teleop (held ~0.5 s)               Space  e-stop (latched)      R   reset
  G      grip toggle (walker)               W / S  push / pull the handles (walker)

EEG is SIMULATED (gaze-driven): the gaze key drives the synthetic Oz signal, the real decoder
and intent rules decide.  Usage:  python standalone/app/interactive.py --world supermarket --robot walker
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import mujoco.viewer  # noqa: E402

from eyesteer_core.control.mux_rules import Twist  # noqa: E402
from standalone.app.runner import DT, ScenarioRunner  # noqa: E402

KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT = 265, 264, 263, 262


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="field", choices=["field", "supermarket"])
    ap.add_argument("--robot", default="field_robot", choices=["field_robot", "walker"])
    a = ap.parse_args(argv)
    scn = {"name": "interactive", "world": a.world, "robot": a.robot, "duration": 1e9, "seed": 1, "snr": 1.0,
           "events": [], "camera": "chase"}
    r = ScenarioRunner(scn)
    r.manual_gaze = 0
    by_slot = {str(m.slot): m.id for m in r.markers}
    state = {"teleop_until": 0.0, "teleop": Twist()}

    def key(k):
        ch = chr(k) if 32 <= k < 127 else ""
        if ch in "12345" and ch in by_slot:
            r.manual_gaze = by_slot[ch]
        elif ch.upper() == "C":
            r.manual_gaze = by_slot.get("CANCEL", 0)
        elif ch == "0":
            r.manual_gaze = 0
        elif k == 32:
            r.estop_pressed = True
            r._handle(r.fsm.on_estop(r.t, True))
        elif ch.upper() == "R":
            r.estop_pressed = False
            r._handle(r.fsm.reset(r.t))
        elif ch.upper() == "G":
            r.manual_grip = not (r.manual_grip if r.manual_grip is not None else True)
        elif ch.upper() == "W":
            r.manual_force = (15.0, 15.0) if r.manual_force[0] <= 0 else (0.0, 0.0)
        elif ch.upper() == "S":
            r.manual_force = (-20.0, -20.0) if r.manual_force[0] >= 0 else (0.0, 0.0)
        elif k in (KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT):
            v = {KEY_UP: 0.3, KEY_DOWN: -0.2}.get(k, 0.0)
            w = {KEY_LEFT: 0.8, KEY_RIGHT: -0.8}.get(k, 0.0)
            state["teleop"], state["teleop_until"] = Twist(v, w), r.t + 0.5
        print(f"[key] gaze={r.manual_gaze} grip={r.manual_grip} force={r.manual_force} -> {r.fsm.status().display}")

    print(__doc__)
    print("SIMULATED EEG (gaze-driven)")
    with mujoco.viewer.launch_passive(r.sim.m, r.sim.d, key_callback=key) as v:
        v.opt.geomgroup[3] = v.opt.geomgroup[4] = 1
        last_status = ""
        while v.is_running():
            t0 = time.time()
            r.manual_teleop = state["teleop"] if r.t < state["teleop_until"] else None
            r.step()
            st = r.fsm.status()
            for m in r.markers:   # MR overlay styling, as in the videos
                ring = [0.35, 0.95, 1.0, 0.95] if m.id == st.goal else ([1, 1, 1, 0.6] if m.id in st.queue else [0, 0, 0, 0])
                r.sim.style_marker(m.id, [1, 1, 1, 0.55], ring)
            s = f"{st.display} queue={st.queue} mux={r.mux_source} cm={r.cm_state} bci={r.bci.status.quality}"
            if s != last_status:
                print(f"t={r.t:6.1f}  {s}")
                last_status = s
            v.sync()
            time.sleep(max(0.0, DT - (time.time() - t0)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

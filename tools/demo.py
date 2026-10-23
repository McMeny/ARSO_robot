#!/usr/bin/env python3
"""Gated startup orchestration (D21, Section 24.4): `make demo-sim` / `make demo-real`.

Launches the system, then waits for the health gates in dependency order (backend -> sensors ->
localization -> Nav2 -> safety -> EEG -> headset), each with a timeout and a specific hint. The
health_monitor keeps the twist_mux motion lock engaged until EVERY required gate is green, so a
half-working system can never drive. Ctrl-C stops everything.
"""
from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time

ORDER = [  # (gate, timeout s, hint when it stays red)
    ("mcu", 30, "MCU link: check /dev/eyesteer_mcu (udev rule, USB cable), `make flash`, or the HIL emulator log"),
    ("lidar", 30, "LiDAR: /scan missing or not ~10 Hz in laser_frame. Check /dev/lidar, the sllidar driver log, the motor spins"),
    ("tf", 30, "TF: map->odom->base_footprint->laser_frame incomplete. Is ros2_control / the bridge publishing odom? AMCL running?"),
    ("localization", 60, "Localization: AMCL not converged. Set the initial pose in RViz (2D Pose Estimate) or drive a little"),
    ("nav2", 60, "Nav2: lifecycle nodes not active. See the controller/planner/bt_navigator logs (params, map, TF)"),
    ("safety", 30, "Safety: collision monitor inactive or no walker grip data (dead-man sensor)"),
    ("muse_lsl", 60, "Muse/LSL: stream not found or not 256 Hz / BAD SIGNAL. Is muselsl streaming? Oz electrode gel/contact? `python3 tools/eeg_check.py stream`"),
    ("bci", 30, "BCI: REAL EEG needs a per-user calibration profile: headset Run -> 'Per-user EEG calibration'"),
    ("frequency_table", 30, "Frequency table hash mismatch between decoder, markers and headset: reload the headset page after `make generate`"),
    ("headset", 300, "Headset: open https://<jetson>:8443 in the Quest browser, accept the consent, calibrate (2 decals), place >= 1 marker; refresh must equal hardware.yaml"),
]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["sim", "real"])
    ap.add_argument("--world", default="supermarket")
    ap.add_argument("--robot", default="walker")
    ap.add_argument("--eeg", default="synthetic")
    ap.add_argument("--backend", default="mujoco_hil")
    ap.add_argument("--map", default="")
    ap.add_argument("--no-launch", action="store_true", help="only watch the gates of an already running system")
    ap.add_argument("--desktop", action="store_true",
                    help="no headset, no Muse: a browser's 2D map is the headset (click a marker = look at it), "
                         "SIMULATED EEG through the real decoder, no on-screen flicker")
    a = ap.parse_args(argv)
    proc = None
    if not a.no_launch:
        if a.mode == "sim":
            cmd = ["ros2", "launch", "eyesteer_bringup", "sim.launch.py", f"world:={a.world}", f"robot:={a.robot}",
                   f"eeg:={a.eeg}", f"backend:={a.backend}", "require_headset:=true", "rviz:=true", "viewer:=true"]
        else:
            cmd = ["ros2", "launch", "eyesteer_bringup", "real.launch.py", f"robot:={a.robot}", f"eeg:={a.eeg}",
                   "require_headset:=true"] + ([f"map:={a.map}"] if a.map else [])
        if a.desktop:                              # the launch files force eeg:=synthetic + require_headset:=false
            cmd.append("demo:=true")
        print("[demo] launching:", " ".join(cmd))
        proc = subprocess.Popen(cmd, stdout=open("/tmp/eyesteer_demo.log", "w"), stderr=subprocess.STDOUT)

    import rclpy
    from rclpy.node import Node

    from eyesteer_interfaces.msg import SystemHealth

    rclpy.init()
    node = Node("demo_orchestrator")
    state = {"h": None}
    node.create_subscription(SystemHealth, "/system/health", lambda m: state.__setitem__("h", m), 10)
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    t0 = time.time()
    try:
        for gate, tmo, hint in ORDER:
            t_gate = time.time()
            shown = False
            while not stop["v"]:
                rclpy.spin_once(node, timeout_sec=0.2)
                h = state["h"]
                comp = next((c for c in h.components if c.name == gate), None) if h else None
                if comp is not None and not comp.required:
                    print(f"[demo]   -  {gate:16s} not required in this mode")
                    break
                if comp is not None and comp.level == 0:
                    print(f"[demo]  OK  {gate:16s} {comp.message}  (+{time.time() - t0:.0f} s)")
                    break
                if time.time() - t_gate > tmo and not shown:
                    print(f"[demo] RED  {gate:16s} {comp.message if comp else 'no health data'}\n         -> {hint}\n         (still waiting; motion stays LOCKED)")
                    shown = True
            if stop["v"]:
                break
        while not stop["v"]:
            rclpy.spin_once(node, timeout_sec=0.5)
            h = state["h"]
            if h and h.motion_unlocked:
                print(f"[demo] ALL REQUIRED GATES GREEN: motion UNLOCKED (+{time.time() - t0:.0f} s). Dashboard: https://<host>:8443/dashboard.html")
                if a.desktop:
                    where = "localhost" if a.mode == "sim" else "<jetson>"
                    print(f"[demo] DESKTOP DEMO: open https://{where}:8443 in a browser"
                          + ("" if a.mode == "sim" else "; 'Place markers' tab: click on the map to place them") +
                          "; 'Run' tab: click a marker or press its number to look at it (C = CANCEL, 0/Esc = look away, "
                          "Space = e-stop). EEG is SIMULATED.")
                break
        last = None
        while not stop["v"]:   # keep watching: report any gate that goes red during the run
            rclpy.spin_once(node, timeout_sec=0.5)
            h = state["h"]
            if h:
                red = tuple(c.name for c in h.components if c.required and c.level != 0)
                if red != last:
                    print(f"[demo] {'red gates: ' + ', '.join(red) if red else 'all gates green'} | motion {'UNLOCKED' if h.motion_unlocked else 'LOCKED'}")
                    last = red
    finally:
        if proc:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                proc.kill()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

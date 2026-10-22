#!/usr/bin/env python3
"""MCU-side latency: 'm' command written to the emulator pty -> MuJoCo wheel speed > 0.05 rad/s.
Stepped exactly like the HIL physics loop (10 ms steps, emulator serviced before/after). Linux."""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "firmware/emulator"))

from mcu_emulator import McuEmulator  # noqa: E402
from standalone.sim.simworld import SimWorld  # noqa: E402


def trial(robot, counts, offset):
    sim = SimWorld("field" if robot == "field_robot" else "supermarket", robot)
    for p in sim.people:
        p.enabled = False
    emu = McuEmulator(robot, link=None, backend=sim, time_fn=lambda: sim.t)
    emu.open()
    fd = os.open(os.ttyname(emu.slave), os.O_RDWR | os.O_NONBLOCK)
    for _ in range(int(0.5 / 0.01)):      # idle, ticking
        emu.service(); sim.step(0.01); emu.service()
    for _ in range(offset):                # random phase vs the 30 Hz PID tick
        emu.service(); sim.step(0.002); emu.service()
    t0 = sim.t
    os.write(fd, f"m {counts} {counts}\r".encode())
    while sim.t - t0 < 1.0:
        emu.service(); sim.step(0.01); emu.service()
        if max(abs(w) for w in sim.wheel_velocities()) > 0.05:
            return 1000 * (sim.t - t0)
    return float("inf")


for robot, counts in (("field_robot", 20), ("walker", 6)):
    lat = sorted(trial(robot, counts, k) for k in range(0, 16, 2))
    print(f"{robot}: m->wheel moving  min {lat[0]:.0f} ms  median {lat[len(lat) // 2]:.0f} ms  max {lat[-1]:.0f} ms")

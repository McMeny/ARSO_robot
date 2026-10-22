#!/usr/bin/env python3
"""HIL coupling check without ROS: MCU emulator (firmware core) driving the MuJoCo wheels, commanded like
diffdrive_arduino ('m' every 33 ms). Prints commanded vs achieved wheel speed and the encoder odometry
error vs ground truth. Linux (pty + g++)."""
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "firmware/emulator"))

from eyesteer_core.control.diffdrive_kinematics import EncoderOdometry, wrap  # noqa: E402
from mcu_emulator import McuEmulator  # noqa: E402
from standalone.sim.simworld import SimWorld  # noqa: E402


def main(robot="field_robot"):
    sim = SimWorld("field" if robot == "field_robot" else "supermarket", robot)
    emu = McuEmulator(robot, link=None, backend=sim, time_fn=lambda: sim.t)
    emu.open()
    fd = os.open(os.ttyname(emu.slave), os.O_RDWR | os.O_NONBLOCK)
    cpr, rate = emu.cpr, emu.rate
    odo = EncoderOdometry(sim.wr, sim.ws)
    x0, y0, yaw0 = sim.pose()
    next_cmd = sim.t
    worst = 0.0
    cur = [0.0, 0.0]
    amax = 0.8 / sim.wr * 1.0     # diff_drive_controller acceleration limit at the wheel (rad/s^2)
    for phase, (wl, wr) in (("forward", (4.0, 4.0)), ("turn", (-3.0, 3.0)), ("arc", (2.0, 5.0))):
        t_end = sim.t + 3.0
        speeds = []
        while sim.t < t_end:
            if sim.t >= next_cmd:
                next_cmd += 1 / 30
                cur = [c + max(-amax / 30, min(amax / 30, w - c)) for c, w in zip(cur, (wl, wr))]
                cl = int(cur[0] / (2 * math.pi / cpr) / rate)
                cr = int(cur[1] / (2 * math.pi / cpr) / rate)
                os.write(fd, f"m {cl} {cr}\r".encode())
                os.write(fd, b"e\r")
            emu.service()
            sim.step(0.002 * 5)
            emu.service()
            try:
                os.read(fd, 4096)
            except BlockingIOError:
                pass
            odo.update(*sim.wheel_angles(), 0.01)
            speeds.append(sim.wheel_velocities())
        tail = speeds[len(speeds) // 2:]
        ml = sum(s[0] for s in tail) / len(tail)
        sl = math.sqrt(sum((s[0] - ml) ** 2 for s in tail) / len(tail))
        x, y, yaw = sim.pose()
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        dx, dy = x - x0, y - y0
        ex = math.hypot(odo.x - (c * dx + s_ * dy), odo.y - (-s_ * dx + c * dy))
        eyaw = abs(wrap(odo.yaw - (yaw - yaw0)))
        worst = max(worst, eyaw)
        print(f"{phase:8s} cmd L {wl:+.1f} rad/s -> mean {ml:+.2f}, ripple sd {sl:.2f} rad/s | odom err {ex:.3f} m, yaw err {eyaw:.3f} rad")
    return worst


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "field_robot")

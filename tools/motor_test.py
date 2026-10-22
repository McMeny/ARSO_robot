#!/usr/bin/env python3
"""Wheel / encoder check WITHOUT ROS (DEPLOY.md bring-up step 2). Wheels OFF THE GROUND.

Spins each wheel forward then backward at low PWM and prints encoder counts, so motor polarity,
encoder direction and wiring can be checked before ROS starts. Also checks the 200 ms watchdog.

  python3 tools/motor_test.py --device /dev/eyesteer_mcu
  python3 tools/motor_test.py --device /dev/eyesteer_mcu --measure-cpr   # turn one wheel exactly 10 times by hand
Expected: "forward" -> counts INCREASE on that wheel; "backward" -> decrease. Fix with
hardware.yaml motor_driver.invert_* / invert_*_encoder, then `make generate firmware flash`.
"""
from __future__ import annotations

import argparse
import sys
import time

import serial  # pyserial


class Mcu:
    def __init__(self, device: str, baud: int = 57600, timeout: float = 1.0):
        self.s = serial.Serial(device, baud, timeout=timeout)
        time.sleep(2.0)   # AVR boards reset when the port opens
        self.s.reset_input_buffer()

    def cmd(self, c: str) -> str:
        self.s.write((c + "\r").encode())
        return self.s.readline().decode(errors="replace").strip()

    def enc(self) -> tuple[int, int]:
        a, b = self.cmd("e").split()[:2]
        return int(a), int(b)


def spin(m: Mcu, wheel: int, pwm: int, secs: float) -> tuple[int, int]:
    m.cmd("r")
    t_end = time.time() + secs
    while time.time() < t_end:
        m.cmd(f"o {pwm if wheel == 0 else 0} {pwm if wheel == 1 else 0}")   # refresh < watchdog
        time.sleep(0.05)
    m.cmd("o 0 0")
    time.sleep(0.6)   # let the wheel coast to a stop before the next test resets the counters
    return m.enc()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/eyesteer_mcu")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--pwm", type=int, default=70)
    ap.add_argument("--secs", type=float, default=1.5)
    ap.add_argument("--measure-cpr", action="store_true")
    ap.add_argument("--measure-ff", action="store_true", help="counts per PID loop at max PWM (feed-forward constant)")
    ap.add_argument("--pid-rate", type=float, default=50.0)
    a = ap.parse_args(argv)
    m = Mcu(a.device, a.baud)
    print("firmware:", m.cmd("v"))
    if a.measure_cpr:
        m.cmd("r")
        input("Mark the LEFT wheel, turn it FORWARD exactly 10 revolutions by hand, then press Enter... ")
        print(f"left counts/rev = {m.enc()[0] / 10:.1f}  -> put this in hardware.yaml encoders.counts_per_rev")
        return 0
    if a.measure_ff:
        pwm = 128
        m.cmd("r")
        t_end = time.time() + 2.0
        while time.time() < t_end:
            m.cmd(f"o {pwm} {pwm}")
            time.sleep(0.05)
        l0, r0 = m.enc()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            m.cmd(f"o {pwm} {pwm}")
            time.sleep(0.05)
        l1, r1 = m.enc()
        dt = time.time() - t0
        m.cmd("o 0 0")
        per_loop = ((l1 - l0) + (r1 - r0)) / 2 / dt / a.pid_rate * (255 / pwm)
        print(f"counts per PID loop at max PWM ~ {per_loop:.0f}  -> hardware.yaml encoders.ff_counts_per_loop_at_max_pwm")
        return 0
    ok = True
    for wheel, name in ((0, "LEFT"), (1, "RIGHT")):
        for sign, label in ((1, "forward"), (-1, "backward")):
            l, r = spin(m, wheel, sign * a.pwm, a.secs)
            counts = (l, r)[wheel]
            other = (l, r)[1 - wheel]
            good = (counts * sign) > 50 and abs(other) < max(20, abs(counts) // 10)
            ok &= good
            print(f"{name:5s} {label:8s}: counts L={l:7d} R={r:7d}  {'OK' if good else 'CHECK WIRING / POLARITY'}")
    # watchdog: command once, then stop sending -> must stop within ~200 ms
    m.cmd("r")
    m.cmd(f"o {a.pwm} {a.pwm}")
    time.sleep(0.6)
    e1 = m.enc()
    time.sleep(0.5)
    e2 = m.enc()
    wd = abs(e2[0] - e1[0]) < 15 and abs(e2[1] - e1[1]) < 15
    status = m.cmd("s")
    print(f"watchdog: counts after timeout changed by {e2[0] - e1[0]}, {e2[1] - e1[1]} -> {'OK' if wd else 'FAIL'}  ({status})")
    ok &= wd
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

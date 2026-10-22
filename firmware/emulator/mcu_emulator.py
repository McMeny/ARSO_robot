#!/usr/bin/env python3
"""MCU emulator on a pseudo-terminal (D20).

Runs the *firmware's own* C core (lib/eyesteer_fw, compiled to a shared library and loaded with
ctypes) behind a raw pty, so anything that talks to /dev/eyesteer_mcu (diffdrive_arduino via
mcu_bridge_node, motor_test.py, preflight) can be pointed at the emulator unchanged.

Backends for the wheels:
  model   first-order DC motor model (no physics engine; protocol / CI tests)
  mujoco  callbacks into a MuJoCo sim: PWM -> wheel velocity actuator, wheel joint angle -> encoder
          counts (used by the mujoco_bridge in backend:=mujoco_hil)

Usage (stand-alone):  python3 firmware/emulator/mcu_emulator.py --link /tmp/eyesteer_mcu --robot walker
Linux/macOS only (pty).
"""
from __future__ import annotations

import argparse
import ctypes
import math
import os
import select
import subprocess
import sys
import threading
import time
import tty
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
ROOT = FW.parent
sys.path.insert(0, str(ROOT))


def build_library(force: bool = False) -> Path:
    out = FW / "build" / "libeyesteer_fw.so"
    src = FW / "lib/eyesteer_fw/eyesteer_fw.cpp"
    if force or not out.exists() or out.stat().st_mtime < src.stat().st_mtime:
        out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["g++", "-std=c++11", "-O2", "-shared", "-fPIC", "-o", str(out), str(src)], check=True)
    return out


class FirmwareCore:
    """ctypes wrapper around lib/eyesteer_fw (identical code to the MCU)."""

    def __init__(self, consts: dict, robot: str):
        self.lib = ctypes.CDLL(str(build_library()))
        L = self.lib
        L.es_state_size.restype = ctypes.c_int
        self.buf = ctypes.create_string_buffer(L.es_state_size())
        self._robot = ctypes.c_char_p(robot.encode())
        L.es_init.argtypes = [ctypes.c_void_p] + [ctypes.c_int32] * 7 + [ctypes.c_uint32, ctypes.c_long, ctypes.c_char_p]
        L.es_feed_char.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_uint32]
        L.es_feed_char.restype = ctypes.c_int
        L.es_take_response.argtypes = [ctypes.c_void_p]
        L.es_take_response.restype = ctypes.c_char_p
        L.es_add_encoder.argtypes = [ctypes.c_void_p, ctypes.c_int32, ctypes.c_int32]
        L.es_pid_step.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        L.es_set_estop_input.argtypes = [ctypes.c_void_p, ctypes.c_uint8]
        L.es_set_walker_sensors.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_uint8]
        L.es_get_pwm.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.es_get_pwm.restype = ctypes.c_int32
        L.es_get_enc.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.es_get_enc.restype = ctypes.c_int32
        c = consts
        L.es_init(self.buf, c["PID_KP"], c["PID_KD"], c["PID_KI"], c["PID_KO"], c["MAX_PWM"], c["MAX_COUNTS_PER_LOOP"],
                  c["MAX_ACCEL_COUNTS_PER_LOOP2"], c["WATCHDOG_MS"], c["BAUD_RATE"], self._robot)

    def feed(self, ch: bytes, now_ms: int):
        if self.lib.es_feed_char(self.buf, ch, ctypes.c_uint32(now_ms & 0xFFFFFFFF)):
            return self.lib.es_take_response(self.buf)
        return None

    def add_encoder(self, dl, dr):
        self.lib.es_add_encoder(self.buf, int(dl), int(dr))

    def pid_step(self, now_ms):
        self.lib.es_pid_step(self.buf, ctypes.c_uint32(now_ms & 0xFFFFFFFF))

    def pwm(self):
        return self.lib.es_get_pwm(self.buf, 0), self.lib.es_get_pwm(self.buf, 1)

    def estop(self, pressed: bool):
        self.lib.es_set_estop_input(self.buf, 1 if pressed else 0)

    def walker(self, fl, fr, grip):
        self.lib.es_set_walker_sensors(self.buf, float(fl), float(fr), 1 if grip else 0)


def read_constants(robot: str) -> dict:
    """Parse the generated firmware header (same numbers the MCU is flashed with)."""
    consts = {}
    for line in (FW / "include" / f"robot_constants_{robot}.h").read_text(encoding="utf-8").splitlines():
        p = line.split()
        if len(p) >= 3 and p[0] == "#define":
            v = p[2].rstrip("f")
            try:
                consts[p[1]] = int(v)
            except ValueError:
                try:
                    consts[p[1]] = float(v)
                except ValueError:
                    consts[p[1]] = p[2].strip('"')
    return consts


class MotorModel:
    """DC motor + gearbox: PWM -> wheel speed with a first-order lag; integrates the wheel angle."""

    def __init__(self, w_max: float, tau: float = 0.08, max_pwm: int = 255):
        self.w_max, self.tau, self.max_pwm = w_max, tau, max_pwm
        self.w = [0.0, 0.0]
        self.angle = [0.0, 0.0]
        self.cmd = [0.0, 0.0]

    def set_wheels(self, wl, wr):
        self.cmd = [wl, wr]

    def step(self, dt):
        for i in range(2):
            self.w[i] += (self.cmd[i] - self.w[i]) * min(1.0, dt / self.tau)
            self.angle[i] += self.w[i] * dt

    def wheel_angles(self):
        return tuple(self.angle)


class McuEmulator:
    def __init__(self, robot: str = "field_robot", link: str | None = "/tmp/eyesteer_mcu", backend=None,
                 cpr: int | None = None, time_fn=time.monotonic):
        self.robot = robot
        self.c = read_constants(robot)
        self.core = FirmwareCore(self.c, robot)
        self.cpr = cpr or self.c["ENCODER_CPR"]
        self.rate = self.c["PID_RATE_HZ"]
        r = self.c["WHEEL_RADIUS_M"]
        # PWM 255 ~ 1.5 x v_max at the wheel (headroom for the PID), like a sized motor
        w_full = self.c["MAX_COUNTS_PER_LOOP"] * self.rate * 2 * math.pi / self.cpr * 1.25
        self.w_full = w_full
        self.backend = backend or MotorModel(w_full, max_pwm=self.c["MAX_PWM"])
        self.time_fn = time_fn
        self.t0 = time_fn()
        self.lock = threading.Lock()
        self.prev_counts = None
        self.frac = [0.0, 0.0]
        self.master = self.slave = None
        self.link = link
        self.running = False
        self.estop_pressed = False
        self.forces = (0.0, 0.0)
        self.grip = True
        self.rx_log: list = []
        self.alive = True        # set False to emulate a crashed / unplugged MCU

    # ------------------------------------------------------------------ pty
    def open(self) -> str:
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        name = os.ttyname(self.slave)
        if self.link:
            try:
                if os.path.islink(self.link) or os.path.exists(self.link):
                    os.unlink(self.link)
            except OSError:
                pass
            os.symlink(name, self.link)
        return self.link or name

    def now_ms(self) -> int:
        return int((self.time_fn() - self.t0) * 1000)

    # ------------------------------------------------------------------ core coupling
    def pid_tick(self, dt: float | None = None):
        """One firmware PID period: encoder deltas in, PWM out -> wheel commands."""
        with self.lock:
            ang = self.backend.wheel_angles()
            counts = [a * self.cpr / (2 * math.pi) for a in ang]
            if self.prev_counts is None:
                self.prev_counts = counts
            d = []
            for i in range(2):
                raw = counts[i] - self.prev_counts[i] + self.frac[i]
                n = int(math.floor(raw)) if raw >= 0 else -int(math.floor(-raw))
                self.frac[i] = raw - n
                d.append(n)
            self.prev_counts = counts
            self.core.add_encoder(d[0], d[1])
            self.core.estop(self.estop_pressed)
            self.core.walker(self.forces[0], self.forces[1], self.grip)
            self.core.pid_step(self.now_ms())
            pl, pr = self.core.pwm()
            wl = pl / self.c["MAX_PWM"] * self.w_full
            wr = pr / self.c["MAX_PWM"] * self.w_full
            self.backend.set_wheels(wl, wr)
            return pl, pr

    def handle_bytes(self, data: bytes) -> bytes:
        out = b""
        with self.lock:
            for b in data:
                r = self.core.feed(bytes([b]), self.now_ms())
                if r is not None:
                    out += r
        return out

    def serve_forever(self, model_dt: float = 0.002):
        """Serial I/O + PID timer (model backend also steps its motor here)."""
        self.running = True
        next_pid = self.time_fn()
        last = self.time_fn()
        buf = b""
        while self.running:
            now = self.time_fn()
            if isinstance(self.backend, MotorModel):
                self.backend.step(now - last)
            last = now
            if now >= next_pid:
                if self.alive:
                    self.pid_tick()
                next_pid += 1.0 / self.rate
            r, _, _ = select.select([self.master], [], [], 0.002)
            if r:
                try:
                    data = os.read(self.master, 256)
                except OSError:
                    continue
                if not self.alive:
                    continue   # crashed MCU: swallow bytes, never answer
                self.rx_log.append(data)
                self.rx_log = self.rx_log[-200:]
                resp = self.handle_bytes(data)
                if resp:
                    os.write(self.master, resp)

    def start(self):
        if self.master is None:
            self.open()
        th = threading.Thread(target=self.serve_forever, daemon=True)
        th.start()
        return th

    def stop(self):
        self.running = False
        if self.link and os.path.islink(self.link):
            try:
                os.unlink(self.link)
            except OSError:
                pass


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="field_robot", choices=["field_robot", "walker"])
    ap.add_argument("--link", default="/tmp/eyesteer_mcu")
    a = ap.parse_args(argv)
    emu = McuEmulator(a.robot, a.link)
    path = emu.open()
    print(f"[mcu_emulator] {a.robot}: firmware core on {path} -> {os.ttyname(emu.slave)} "
          f"(baud {emu.c['BAUD_RATE']}, PID {emu.rate} Hz, watchdog {emu.c['WATCHDOG_MS']} ms)", flush=True)
    try:
        emu.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        emu.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

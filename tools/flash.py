#!/usr/bin/env python3
"""`make flash`: build + upload the firmware for the board and robot in config/hardware.yaml.
Port: /dev/eyesteer_mcu (udev symlink) if present, else PlatformIO auto-detection (or --port)."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main(argv=None) -> int:
    hw = yaml.safe_load((ROOT / "config/hardware.yaml").read_text())
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    board = hw["mcu"]["board"]
    env = board + ("_walker" if hw["robot"] == "walker" else "")
    if env == "uno_walker":
        print("The walker build does not fit the Uno's interrupt pins: use mega2560 / teensy41 / esp32.")
        return 2
    port = a.port or (hw["mcu"]["device"] if os.path.exists(hw["mcu"]["device"]) else None)
    subprocess.run([sys.executable, str(ROOT / "tools/generate_models.py")], check=True)
    cmd = [sys.executable, "-m", "platformio", "run", "-d", str(ROOT / "firmware"), "-e", env, "-t", "upload"]
    if port:
        cmd += ["--upload-port", port]
    print("[flash]", " ".join(cmd))
    if a.dry_run:
        return 0
    r = subprocess.run(cmd)
    if r.returncode == 0:
        print("[flash] done. Next (wheels OFF the ground):  python3 tools/motor_test.py --device", port or hw["mcu"]["device"])
    return r.returncode


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Real-hardware EEG checks (Section 24.6), each a single command with PASS/FAIL output.

  python3 tools/eeg_check.py stream            # LSL stream found, channel map, ~256 Hz, quality, 10 s Oz spectrum
  python3 tools/eeg_check.py alpha             # 20 s eyes open, 20 s eyes closed -> Oz alpha must rise (electrode + mapping)
  python3 tools/eeg_check.py ssvep             # headset shows each marker in turn (cue via ROS) -> CCA winner must match
Add --emulate to run against a local Muse LSL emulator (CI / rehearsal; prints SIMULATED EEG).
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_ws/src/eyesteer_bci"))

from eyesteer_core import config  # noqa: E402
from eyesteer_core.bci.quality import assess  # noqa: E402
from eyesteer_core.bci.ssvep_cca import SSVEPDecoder  # noqa: E402


def spec():
    e = config.hardware()["eeg"]
    return e, e["tools"][e["source_tool"]]


def start_emulator():
    from eyesteer_bci.muse_emulator import MuseEmulator, default_freqs
    emu = MuseEmulator(freqs=default_freqs(), seed=3)
    emu.start()
    print("[eeg_check] using the Muse LSL EMULATOR: SIMULATED EEG")
    return emu


HOST = None   # pin the producer host (--emulate: this machine; else eeg.stream_host if set)


def connect(timeout=10.0):
    import pylsl
    e, s = spec()
    res = [r for r in pylsl.resolve_byprop("name", s["stream_name"], 0, min(timeout, 4.0)) if r.type() == s["stream_type"]]
    host = HOST or e.get("stream_host")
    if host:
        res = [r for r in res if r.hostname() == host]
    if not res:
        print(f"FAIL  no LSL stream name='{s['stream_name']}' type='{s['stream_type']}' (is {e['source_tool']} streaming?)")
        sys.exit(1)
    info = res[0]
    # clock-sync only: raw sample timestamps give the true producer rate (dejitter would smooth it)
    inlet = pylsl.StreamInlet(info, max_buflen=60, processing_flags=pylsl.proc_clocksync)
    inlet.open_stream(timeout=5.0)
    return inlet.info(timeout=5.0), inlet


def record(inlet, seconds):
    xs, ts = [], []
    t_end = time.time() + seconds
    while time.time() < t_end:
        c, t = inlet.pull_chunk(timeout=0.2)
        if c:
            xs += c
            ts += t
    return np.asarray(xs, float), np.asarray(ts, float)


def band(x, fs, lo, hi):
    from scipy.signal import welch
    f, p = welch(x - x.mean(), fs=fs, nperseg=int(fs * 2))
    return float(p[(f >= lo) & (f <= hi)].mean())


def cmd_stream(a):
    e, s = spec()
    info, inlet = connect()
    n = info.channel_count()
    ch = info.desc().child("channels").child("channel")
    labels = []
    for _ in range(n):
        labels.append(ch.child_value("label"))
        ch = ch.next_sibling()
    print(f"stream: {info.name()} / {info.type()}  {n} ch  nominal {info.nominal_srate()} Hz  labels {labels}")
    ok = n >= 5
    x, t = record(inlet, 10.0)
    rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else 0
    oz = e["channel_map"]["Oz"]
    q = assess(x[-256:], 256, oz)
    print(f"measured rate {rate:.1f} Hz (expect 256 +/- 5)  -> {'OK' if abs(rate - 256) <= 5 else 'BAD'}")
    print(f"Oz (channel {oz} = '{labels[oz] if oz < len(labels) else '?'}') quality: {q.label}  std {q.oz_std:.1f} uV")
    from scipy.signal import welch
    f, p = welch(x[:, oz] - x[:, oz].mean(), fs=256, nperseg=512)
    print("Oz spectrum 1-40 Hz (10 s):")
    m = (f >= 1) & (f <= 40)
    pmax = p[m].max()
    for fi, pi in zip(f[m][::2], p[m][::2]):
        print(f"  {fi:5.1f} Hz |{'#' * int(40 * pi / pmax)}")
    ok = ok and abs(rate - 256) <= 5 and q.ok
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def cmd_alpha(a, emu=None):
    e, s = spec()
    _, inlet = connect()
    oz = e["channel_map"]["Oz"]
    input("Eyes OPEN, relax, look at a fixed point for 20 s. Press Enter to start...") if not a.emulate else None
    xo, _ = record(inlet, a.seconds)
    if emu:
        emu.sim.eyes_closed = True
    input("Now CLOSE your eyes for 20 s (we will beep / tell you). Press Enter, then close them...") if not a.emulate else None
    xc, _ = record(inlet, a.seconds)
    if emu:
        emu.sim.eyes_closed = False
    po, pc = band(xo[:, oz], 256, 8, 12), band(xc[:, oz], 256, 8, 12)
    ratio = pc / max(po, 1e-12)
    print(f"Oz alpha (8-12 Hz) power: eyes open {po:.2f}, eyes closed {pc:.2f}, ratio {ratio:.2f} (need >= 1.5)")
    ok = ratio >= 1.5
    print("RESULT:", "PASS" if ok else "FAIL  (check the Oz cup electrode contact / gel and the AUX channel mapping)")
    return 0 if ok else 1


def cmd_ssvep(a, emu=None):
    """Each marker for 10 s: the headset shows the cued marker (via /bci/calibration_cue), we check CCA."""
    e, s = spec()
    table = config.frequency_table()
    freqs = {i + 1: sl["frequency_exact"] for i, sl in enumerate(table["slots"])}
    _, inlet = connect()
    oz = e["channel_map"]["Oz"]
    cue = None
    try:
        import rclpy
        from std_msgs.msg import Int32
        rclpy.init()
        node = rclpy.create_node("ssvep_smoke_test")
        pub = node.create_publisher(Int32, "/bci/calibration_cue", 10)
        cue = lambda i: pub.publish(Int32(data=i))  # noqa: E731
    except Exception:
        print("(ROS not available: show each marker manually when asked)")
    hits = 0
    for mid, f in freqs.items():
        if cue:
            cue(mid)
        if emu:
            emu.gaze = mid
        if not cue and not emu:
            input(f"Look at marker slot {mid} ({f:.2f} Hz) and press Enter...")
        time.sleep(1.0)
        x, _ = record(inlet, a.seconds)
        dec = SSVEPDecoder(list(freqs.values()), list(freqs.keys()))
        res = dec.push(x[:, [oz]])
        votes = [r.best_id for r in res]
        win = max(set(votes), key=votes.count) if votes else None
        ok = win == mid
        hits += ok
        print(f"  marker {mid} {f:6.3f} Hz -> CCA winner {win}  ({votes.count(mid) if votes else 0}/{len(votes)} windows)  {'OK' if ok else 'MISS'}")
    if cue:
        cue(-2)
    if emu:
        emu.gaze = None
    ok = hits >= len(freqs) - 1
    print(f"RESULT: {'PASS' if ok else 'FAIL'} ({hits}/{len(freqs)} markers decoded)")
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("check", choices=["stream", "alpha", "ssvep"])
    ap.add_argument("--emulate", action="store_true")
    ap.add_argument("--seconds", type=float, default=None)
    a = ap.parse_args(argv)
    if a.seconds is None:
        a.seconds = {"stream": 10.0, "alpha": 20.0, "ssvep": 10.0}[a.check]
    global HOST
    emu = start_emulator() if a.emulate else None
    if emu:
        import socket
        HOST = socket.gethostname()
    if emu:
        time.sleep(1.0)
    rc = {"stream": lambda: cmd_stream(a), "alpha": lambda: cmd_alpha(a, emu), "ssvep": lambda: cmd_ssvep(a, emu)}[a.check]()
    if emu:
        emu.running = False
    return rc


if __name__ == "__main__":
    sys.exit(main())

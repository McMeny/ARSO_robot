"""Muse LSL emulator (D20, Section 24.2).

A real pylsl outlet with the configured tool's exact stream name, type, channel labels/order, sample
rate and units (incl. the AUX = Oz cup electrode). Data come from:
  synthetic  eyesteer_core.bci.eeg_sim driven by a gaze source (/bci/gaze from the headset's aimed
             marker, the keyboard, or the scenario_player)  -> always labeled SIMULATED EEG
  replay     a recorded session (.csv: t,TP9,AF7,AF8,TP10,AUX  or .xdf via pyxdf), looped
             -> labeled REPLAYED EEG
ssvep_node cannot tell this stream from the real one; honesty labels come from the launch argument.

Failure injection: /sim/eeg_fault (std_msgs/String): "drop" (outlet stops pushing), "flat" (Oz flat-line),
"ok". Also usable without ROS:  python3 -m eyesteer_bci.muse_emulator --gaze 3
"""
from __future__ import annotations

import argparse
import csv
import threading
import time
from pathlib import Path

from eyesteer_bci import _root  # noqa: F401

import numpy as np
import pylsl

from eyesteer_core import config
from eyesteer_core.bci.eeg_sim import SyntheticEEG


def stream_spec(tool: str | None = None) -> dict:
    eeg = config.hardware()["eeg"]
    return dict(eeg["tools"][tool or eeg["source_tool"]])


def make_outlet(spec: dict, source_id: str = "eyesteer-muse-emulator") -> pylsl.StreamOutlet:
    info = pylsl.StreamInfo(spec["stream_name"], spec["stream_type"], len(spec["channels"]), spec["fs"],
                            pylsl.cf_float32, source_id)
    chans = info.desc().append_child("channels")
    for label in spec["channels"]:
        c = chans.append_child("channel")
        c.append_child_value("label", label)
        c.append_child_value("unit", spec.get("units", "microvolts"))
        c.append_child_value("type", "EEG")
    info.desc().append_child_value("manufacturer", "Muse (emulated)")
    return pylsl.StreamOutlet(info, chunk_size=12, max_buffered=360)


def load_replay(path: str) -> np.ndarray:
    p = Path(path)
    if p.suffix.lower() == ".xdf":
        import pyxdf
        streams, _ = pyxdf.load_xdf(str(p))
        eeg = next(s for s in streams if s["info"]["type"][0].upper() == "EEG")
        return np.asarray(eeg["time_series"], float)[:, :5]
    rows = []
    with open(p, newline="") as f:
        for r in csv.reader(f):
            try:
                rows.append([float(v) for v in r[1:6]])
            except ValueError:
                continue   # header
    return np.asarray(rows, float)


class MuseEmulator:
    def __init__(self, tool: str | None = None, mode: str = "synthetic", replay_file: str | None = None,
                 snr: float | None = None, seed: int = 7, freqs: dict | None = None,
                 source_id: str = "eyesteer-muse-emulator"):
        self.spec = stream_spec(tool)
        self.fs = self.spec["fs"]
        self.mode = mode
        b = config.bci()
        self.sim = SyntheticEEG(fs=self.fs, snr=snr if snr is not None else b["synthetic"]["snr"], seed=seed)
        self.replay = load_replay(replay_file) if mode == "replay" else None
        self.replay_i = 0
        self.freqs = freqs or {}
        self.gaze = None           # marker id or None
        self.engagement = 0.5
        self.fault = "ok"
        self.outlet = None
        self.source_id = source_id
        self.running = False
        self.pushed = 0

    def open(self):
        self.outlet = make_outlet(self.spec, self.source_id)

    def chunk(self, n: int) -> np.ndarray:
        if self.mode == "replay":
            idx = (self.replay_i + np.arange(n)) % len(self.replay)
            self.replay_i = int(idx[-1] + 1)
            x = self.replay[idx]
        else:
            f = self.freqs.get(self.gaze) if self.gaze else None
            x = self.sim.generate(n, f, self.engagement)
        if self.fault == "flat":
            x = x.copy()
            x[:, 4] = 0.0
        return x.astype(np.float32)

    def run(self):
        """Push 8-sample chunks paced by the LSL clock (timestamps = sample times)."""
        self.running = True
        n = 8
        t_next = pylsl.local_clock()
        while self.running:
            now = pylsl.local_clock()
            if now < t_next:
                time.sleep(min(0.01, t_next - now))
                continue
            x = self.chunk(n)
            if self.fault != "drop":
                stamp = t_next + (n - 1) / self.fs
                self.outlet.push_chunk(x.tolist(), stamp)
                self.pushed += n
            t_next += n / self.fs
            if pylsl.local_clock() - t_next > 1.0:    # fell far behind (suspended): resync
                t_next = pylsl.local_clock()

    def start(self):
        if self.outlet is None:
            self.open()
        th = threading.Thread(target=self.run, daemon=True)
        th.start()
        return th


def default_freqs() -> dict:
    """marker id -> frequency for the default layouts (ids are shared by both worlds)."""
    from eyesteer_core.world import load_markers
    return {m.id: m.frequency for m in load_markers("field_default")}


def main_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32, Int32, String

    from eyesteer_interfaces.msg import MarkerArray

    rclpy.init()
    node = Node("muse_emulator")
    mode = node.declare_parameter("mode", "synthetic").value
    replay_file = node.declare_parameter("replay_file", "").value
    tool = node.declare_parameter("tool", "").value or None
    snr = node.declare_parameter("snr", -1.0).value
    sid = node.declare_parameter("source_id", "eyesteer-muse-emulator").value
    emu = MuseEmulator(tool, mode, replay_file or None, None if snr < 0 else snr, freqs=default_freqs(), source_id=sid)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    def on_markers(m):
        emu.freqs = {mk.id: mk.frequency for mk in m.markers}

    def on_gaze(m):
        emu.gaze = m.data if m.data > 0 else None

    node.create_subscription(Int32, "/bci/gaze", on_gaze, 10)
    node.create_subscription(Float32, "/bci/engagement", lambda m: setattr(emu, "engagement", m.data), 10)
    node.create_subscription(String, "/sim/eeg_fault", lambda m: setattr(emu, "fault", m.data), 10)
    node.create_subscription(MarkerArray, "/markers", on_markers, latched)
    src = node.create_publisher(String, "/bci/eeg_source", latched)
    src.publish(String(data="REPLAYED" if mode == "replay" else "SIMULATED"))
    emu.start()
    node.get_logger().info(f"Muse LSL emulator: stream '{emu.spec['stream_name']}' ({emu.spec['stream_type']}, "
                           f"{emu.spec['channels']}, {emu.fs} Hz) mode={mode}  -> {'REPLAYED' if mode == 'replay' else 'SIMULATED'} EEG")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    emu.running = False
    rclpy.try_shutdown()


def main_cli(argv=None):
    ap = argparse.ArgumentParser(description="Muse LSL emulator without ROS (preflight / tests)")
    ap.add_argument("--gaze", type=int, default=0)
    ap.add_argument("--mode", default="synthetic")
    ap.add_argument("--replay", default=None)
    ap.add_argument("--seconds", type=float, default=0)
    a = ap.parse_args(argv)
    emu = MuseEmulator(mode=a.mode, replay_file=a.replay, freqs=default_freqs())
    emu.gaze = a.gaze or None
    emu.start()
    print(f"[muse_emulator] streaming '{emu.spec['stream_name']}' SIMULATED EEG, gaze={a.gaze}", flush=True)
    t0 = time.time()
    while not a.seconds or time.time() - t0 < a.seconds:
        time.sleep(0.2)
    emu.running = False


if __name__ == "__main__":
    main_cli()

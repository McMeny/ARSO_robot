"""ssvep_node: LSL EEG in -> /bci/status (4 Hz) + /bci/selection events. Thin wrapper over
eyesteer_core.bci.BciPipeline (the same code as the standalone sim).

LSL handling (INTERFACES.md): resolve by name+type (timeout from hardware.yaml), inlet with
clock-sync + dejitter post-processing (LSL time_correction), 4 s buffer, reconnect after 2 s without
samples (status BAD SIGNAL "stream lost" meanwhile, no selections), measured sample rate published.
Calibration service /bci/calibrate (eyesteer_interfaces/Calibrate): cues on /bci/calibration_cue.
"""
from __future__ import annotations

import threading
import time

from eyesteer_bci import _root  # noqa: F401

import numpy as np
import pylsl
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

from eyesteer_core import config
from eyesteer_core.bci.calibration import fit_profile, load_profile, save_profile, synthetic_calibration
from eyesteer_core.bci.pipeline import BciPipeline
from eyesteer_core.world import Marker, load_markers

from eyesteer_interfaces.msg import BciStatus, MarkerArray, Selection
from eyesteer_interfaces.srv import Calibrate


class SsvepNode(Node):
    def __init__(self):
        super().__init__("ssvep_node")
        p = self.declare_parameter
        hw = config.hardware()["eeg"]
        tool = p("tool", hw["source_tool"]).value
        spec = hw["tools"][tool]
        self.stream_name = p("stream_name", spec["stream_name"]).value
        self.stream_type = p("stream_type", spec["stream_type"]).value
        self.source_label = p("source_label", "REAL").value          # SIMULATED | REPLAYED | REAL (D18)
        self.user = p("user", "default").value
        self.resolve_timeout = p("resolve_timeout", float(hw["resolve_timeout_s"])).value
        self.stream_host = p("stream_host", hw.get("stream_host") or "").value
        self.stream_source_id = p("stream_source_id", "").value   # sim: exactly this launch's emulator
        self.channel_map = dict(hw["channel_map"])
        self.cfg = config.bci()
        self.table = config.frequency_table()
        self.profile = load_profile(self.user, self.cfg["calibration"]["profile_dir"])
        self.markers = load_markers("field_default", self.table)
        self.lock = threading.Lock()
        self._build_pipeline()
        self.inlet = None
        self.last_sample_wall = 0.0
        self.rate_window = []
        self.measured_rate = 0.0
        self.recording = None    # calibration buffer
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(BciStatus, "/bci/status", 10)
        self.pub_sel = self.create_publisher(Selection, "/bci/selection", 10)
        self.pub_cue = self.create_publisher(Int32, "/bci/calibration_cue", 10)
        self.create_subscription(MarkerArray, "/markers", self.on_markers, latched, callback_group=cb)
        self.create_subscription(String, "/bci/eeg_source", lambda m: setattr(self, "source_label", m.data), latched)
        self.create_service(Calibrate, "/bci/calibrate", self.on_calibrate, callback_group=cb)
        self.create_timer(0.25, self.publish_status, callback_group=cb)
        self.running = True
        threading.Thread(target=self.acquire, daemon=True).start()
        self.get_logger().info(f"ssvep_node: LSL '{self.stream_name}'/{self.stream_type}, source={self.source_label}, "
                               f"profile={'loaded' if self.profile else 'defaults'}, table {self.table['hash']}")

    def _build_pipeline(self):
        with self.lock:
            self.pipe = BciPipeline(self.markers, self.cfg, source=self.source_label, channel_map=self.channel_map,
                                    profile=self.profile)

    def on_markers(self, m: MarkerArray):
        mk = [Marker(id=x.id, label=x.label, slot=x.slot, x=x.position.x, y=x.position.y, z=x.position.z,
                     facing=x.facing, cancel=x.cancel, frequency=x.frequency) for x in m.markers]
        if [(a.id, round(a.frequency, 3)) for a in mk] != [(a.id, round(a.frequency, 3)) for a in self.markers]:
            self.markers = mk
            self._build_pipeline()
            self.get_logger().info(f"marker set updated: {[(a.label, round(a.frequency, 2)) for a in mk]}")

    # ------------------------------------------------------------------ acquisition
    def connect(self) -> bool:
        res = pylsl.resolve_byprop("name", self.stream_name, 0, self.resolve_timeout)
        res = [r for r in res if r.type() == self.stream_type]
        if self.stream_host:   # several Muse streams on the network (laptop + Jetson): pin the producer
            res = [r for r in res if r.hostname() == self.stream_host]
        if self.stream_source_id:
            res = [r for r in res if r.source_id() == self.stream_source_id]
        if not res:
            return False
        info = res[0]
        self.inlet = pylsl.StreamInlet(info, max_buflen=4, processing_flags=pylsl.proc_clocksync | pylsl.proc_dejitter)
        self.inlet.open_stream(timeout=self.resolve_timeout)
        self.get_logger().info(f"LSL connected: {info.name()} {info.type()} {info.channel_count()} ch @ {info.nominal_srate()} Hz "
                               f"(source_id {info.source_id()}, host {info.hostname()})")
        self.last_sample_wall = time.monotonic()
        return True

    def acquire(self):
        while self.running and rclpy.ok():
            if self.inlet is None:
                if not self.connect():
                    with self.lock:
                        self.pipe.force_bad = True
                    continue
            try:
                chunk, ts = self.inlet.pull_chunk(timeout=0.1, max_samples=256)
            except pylsl.LostError:
                chunk, ts = [], []
                self.inlet = None
            now = time.monotonic()
            if chunk:
                x = np.asarray(chunk, float)
                # sample rate from the LSL sample timestamps (immune to delivery jitter / CPU load)
                self.rate_window.append((float(ts[0]), float(ts[-1]), len(x)))
                self.rate_window = [w for w in self.rate_window if ts[-1] - w[1] < 3.0]
                span = self.rate_window[-1][1] - self.rate_window[0][0]
                if span > 1.0:
                    self.measured_rate = (sum(w[2] for w in self.rate_window) - 1) / span
                self.last_sample_wall = now
                if self.recording is not None:
                    self.recording.append(x)
                with self.lock:
                    self.pipe.force_bad = False
                    sels = self.pipe.push(x, float(ts[-1]))
                for s in sels:
                    self.publish_selection(s)
            elif now - self.last_sample_wall > 2.0:
                with self.lock:
                    self.pipe.force_bad = True
                if self.inlet is not None:
                    self.get_logger().warn("LSL stream silent for 2 s: reconnecting (no selections meanwhile)")
                    self.inlet = None
                self.measured_rate = 0.0

    def publish_selection(self, s):
        lab = next((m.label for m in self.markers if m.id == s.marker_id), str(s.marker_id))
        msg = Selection(marker_id=int(s.marker_id), label=lab, confidence=float(s.confidence), rho=float(s.rho),
                        margin=float(s.margin), source=self.source_label)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_sel.publish(msg)
        self.get_logger().info(f"SELECTION {lab} (rho {s.rho:.2f}, margin {s.margin:.2f}) [{self.source_label} EEG]")

    def publish_status(self):
        with self.lock:
            st = self.pipe.status
            m = BciStatus()
            m.stamp = self.get_clock().now().to_msg()
            m.marker_ids = [int(i) for i in self.pipe.ids]
            m.labels = list(self.pipe.labels)
            m.frequencies = [float(x.frequency) for x in self.pipe.markers]
            m.rhos = [float(r) for r in st.rhos]
            m.candidate = int(st.candidate) if st.candidate is not None else -1
            m.dwell = float(st.dwell)
            m.engagement_z = float(st.engagement_z)
            m.gate = self.cfg["engagement"]["gate"]
            m.signal_ok = bool(st.signal_ok) and not self.pipe.force_bad
            m.quality = "BAD SIGNAL (stream lost)" if self.pipe.force_bad else st.quality
            m.source = self.source_label
            m.threshold = float(self.pipe.intent.theta)
            m.no_control = bool(st.no_control)
            m.sample_rate = float(self.measured_rate)
            m.table_hash = self.table["hash"]
            m.profile_loaded = self.profile is not None
        self.pub_status.publish(m)

    # ------------------------------------------------------------------ calibration (Section 9)
    def on_calibrate(self, req, resp):
        user = req.user or self.user
        cal = self.cfg["calibration"]
        freqs = {m.id: m.frequency for m in self.markers}
        try:
            if req.synthetic:
                prof = synthetic_calibration(freqs, fs=self.cfg["fs"], per_marker_s=cal["per_marker_s"], rest_s=cal["rest_s"])
            else:
                trials = {}
                for m in self.markers:
                    self.pub_cue.publish(Int32(data=int(m.id)))      # headset highlights this marker, dims the others
                    time.sleep(1.0)
                    self.recording = []
                    time.sleep(cal["per_marker_s"])
                    trials[m.id] = np.concatenate(self.recording) if self.recording else np.zeros((0, 5))
                self.pub_cue.publish(Int32(data=-1))                 # "look at nothing"
                time.sleep(1.0)
                self.recording = []
                time.sleep(cal["rest_s"])
                rest = np.concatenate(self.recording) if self.recording else np.zeros((0, 5))
                self.recording = None
                self.pub_cue.publish(Int32(data=-2))                 # done
                prof = fit_profile(trials, rest, freqs, fs=self.cfg["fs"], oz_index=self.channel_map["Oz"])
            path = save_profile(user, prof, cal["profile_dir"])
            self.profile = prof
            self._build_pipeline()
            resp.success = True
            resp.message = f"profile saved to {path}"
            resp.rho_threshold = float(prof["rho_threshold"])
            resp.margin_threshold = float(prof["margin_threshold"])
            resp.dwell_windows = int(prof["dwell_windows"])
            resp.window_accuracy = float(prof["window_accuracy"])
        except Exception as e:  # report, never crash the node
            self.recording = None
            resp.success, resp.message = False, f"calibration failed: {e}"
        return resp


def main():
    rclpy.init()
    n = SsvepNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    n.running = False
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

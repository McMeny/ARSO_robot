"""lsl_recorder: writes an XDF file (LabRecorder-compatible: XDF 1.0 FileHeader/StreamHeader/Samples/
ClockOffset/StreamFooter chunks) with the EEG stream + an 'eyesteer_events' string marker stream.

Events = BCI selections, robot state changes, and the headset's flicker-onset / marker events
(/xr/events, std_msgs/String JSON from the WebXR app). Timestamps are LSL local_clock; per-stream
clock offsets (LSL time_correction) are stored as ClockOffset chunks so real demo-day sessions become
synchronised experimental data (Section 24.8).
"""
from __future__ import annotations

import struct
import threading
import time
from pathlib import Path

from eyesteer_bci import _root  # noqa: F401

import pylsl

from eyesteer_core import config


def _varlen(n: int) -> bytes:
    if n < 256:
        return bytes([1]) + struct.pack("<B", n)
    if n < 2 ** 32:
        return bytes([4]) + struct.pack("<I", n)
    return bytes([8]) + struct.pack("<Q", n)


class XdfWriter:
    def __init__(self, path: str):
        self.f = open(path, "wb")
        self.lock = threading.Lock()
        self.f.write(b"XDF:")
        self._chunk(1, b'<?xml version="1.0"?><info><version>1.0</version></info>')

    def _chunk(self, tag: int, content: bytes):
        body = struct.pack("<H", tag) + content
        with self.lock:
            self.f.write(_varlen(len(body)) + body)
            self.f.flush()

    def stream_header(self, sid: int, name: str, typ: str, n_ch: int, srate: float, fmt: str, labels=()):
        ch = "".join(f"<channel><label>{lab}</label></channel>" for lab in labels)
        xml = (f'<?xml version="1.0"?><info><name>{name}</name><type>{typ}</type><channel_count>{n_ch}</channel_count>'
               f"<nominal_srate>{srate}</nominal_srate><channel_format>{fmt}</channel_format>"
               f"<created_at>{pylsl.local_clock()}</created_at><desc><channels>{ch}</channels></desc></info>")
        self._chunk(2, struct.pack("<I", sid) + xml.encode())

    def samples_float(self, sid: int, ts, rows):
        out = bytearray(struct.pack("<I", sid)) + _varlen(len(rows))
        for t, r in zip(ts, rows):
            out += bytes([8]) + struct.pack("<d", t) + struct.pack(f"<{len(r)}f", *r)
        self._chunk(3, bytes(out))

    def samples_string(self, sid: int, t: float, s: str):
        b = s.encode()
        out = struct.pack("<I", sid) + _varlen(1) + bytes([8]) + struct.pack("<d", t) + _varlen(len(b)) + b
        self._chunk(3, out)

    def clock_offset(self, sid: int, collection_time: float, offset: float):
        self._chunk(4, struct.pack("<Idd", sid, collection_time, offset))

    def footer(self, sid: int, first: float, last: float, count: int):
        xml = (f'<?xml version="1.0"?><info><first_timestamp>{first}</first_timestamp><last_timestamp>{last}'
               f"</last_timestamp><sample_count>{count}</sample_count></info>")
        self._chunk(6, struct.pack("<I", sid) + xml.encode())

    def close(self):
        self.f.close()


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    from eyesteer_interfaces.msg import RobotStatus, Selection

    rclpy.init()
    node = Node("lsl_recorder")
    out = node.declare_parameter("out", "eyesteer_session.xdf").value
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    eeg = config.hardware()["eeg"]
    spec = eeg["tools"][eeg["source_tool"]]
    w = XdfWriter(out)
    w.stream_header(1, "eyesteer_events", "Markers", 1, 0.0, "string")
    stats = {1: [None, None, 0], 2: [None, None, 0]}
    last_state = [None]

    def event(s: str):
        t = pylsl.local_clock()
        w.samples_string(1, t, s)
        st = stats[1]
        st[0] = st[0] if st[0] is not None else t
        st[1], st[2] = t, st[2] + 1

    def on_status(m):
        if m.display != last_state[0]:
            last_state[0] = m.display
            event(f"state:{m.display}")

    node.create_subscription(Selection, "/bci/selection", lambda m: event(f"selection:{m.marker_id}:{m.label}:{m.source}"), 10)
    node.create_subscription(RobotStatus, "/robot/status", on_status, 10)
    node.create_subscription(String, "/xr/events", lambda m: event(f"xr:{m.data}"), 10)
    running = [True]

    def pull():
        inlet = None
        while running[0]:
            if inlet is None:
                res = pylsl.resolve_byprop("name", spec["stream_name"], 1, 2.0)
                if not res:
                    continue
                inlet = pylsl.StreamInlet(res[0], max_buflen=10)
                w.stream_header(2, spec["stream_name"], spec["stream_type"], len(spec["channels"]), spec["fs"],
                                "float32", spec["channels"])
                last_off = 0.0
            chunk, ts = inlet.pull_chunk(timeout=0.2)
            if chunk:
                w.samples_float(2, ts, chunk)
                st = stats[2]
                st[0] = st[0] if st[0] is not None else ts[0]
                st[1], st[2] = ts[-1], st[2] + len(ts)
            if time.time() - last_off > 5.0:
                last_off = time.time()
                w.clock_offset(2, pylsl.local_clock(), inlet.time_correction(timeout=1.0))

    th = threading.Thread(target=pull, daemon=True)
    th.start()
    node.get_logger().info(f"recording EEG '{spec['stream_name']}' + events to {out}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    running[0] = False
    th.join(timeout=2)
    for sid, (a, b, n) in stats.items():
        if a is not None:
            w.footer(sid, a, b, n)
    w.close()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

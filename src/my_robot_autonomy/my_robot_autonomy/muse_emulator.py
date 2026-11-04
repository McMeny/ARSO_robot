"""A stand-in for the Muse: publishes an LSL stream laid out like the real one
(5 channels - TP9, AF7, AF8, TP10, Right AUX = Oz - at 256 Hz), so the real decoder
is tested end to end without a headband.

The Oz channel carries background EEG (drifting noise + an alpha rhythm) plus an SSVEP
at the frequency of every tagged hazard the simulated user "notices":
  - a hazard is in view if it's within view_half_angle of where the user is looking
    (the headset's /head_pose; with no headset, straight ahead of the walker)
  - the first time each hazard comes into view, the user notices it with probability
    detect_prob (decided once per pass, so it doesn't flip back and forth)
The SSVEP check's test patch (/test_stimulus) is always "seen".
"""
import json
import math
import random
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from walker_interfaces.msg import Hazard, HazardArray

try:
    import pylsl
except ImportError:
    pylsl = None

FS = 256
CHUNK = 8
LABELS = ['TP9', 'AF7', 'AF8', 'TP10', 'Right AUX']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class MuseEmulator(Node):
    def __init__(self):
        super().__init__('muse_emulator')
        p = self.declare_parameter
        self.detect_prob = p('detect_prob', 0.8).value
        self.amplitude = p('ssvep_amplitude_uv', 3.0).value
        self.noise_uv = p('noise_uv', 8.0).value
        self.view_half_angle = math.radians(p('view_half_angle_deg', 50.0).value)

        self.hazards = []
        self.head = None            # (x, y, yaw, time)
        self.noticed = {}           # hazard id -> True / False, decided on first view
        self.stimulus = 0.0         # test patch frequency, 0 = off
        self.active = []            # frequencies currently "seen" (written by ROS, read by the LSL thread)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(PoseStamped, 'head_pose', self.on_head, 10)
        self.create_subscription(String, 'test_stimulus', self.on_stimulus, 10)
        self.create_timer(0.1, self.update_seen)

        if pylsl is None:
            self.get_logger().error('pylsl is not installed - run: pip install pylsl')
            return
        info = pylsl.StreamInfo('Muse', 'EEG', len(LABELS), FS, 'float32', 'muse-emulator')
        channels = info.desc().append_child('channels')
        for label in LABELS:
            ch = channels.append_child('channel')
            ch.append_child_value('label', label)
            ch.append_child_value('unit', 'microvolts')
        self.outlet = pylsl.StreamOutlet(info, CHUNK)
        threading.Thread(target=self.stream, daemon=True).start()
        self.get_logger().info(f'Emulated Muse streaming on LSL (detect_prob {self.detect_prob})')

    def on_head(self, msg):
        self.head = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation), time.time())

    def on_stimulus(self, msg):
        data = json.loads(msg.data)
        self.stimulus = float(data.get('freq', 0)) if data.get('on') else 0.0

    def gaze(self):
        if self.head is not None and time.time() - self.head[3] < 1.0:
            return self.head[:3]
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def update_seen(self):
        gaze = self.gaze()
        seen = [self.stimulus] if self.stimulus > 0 else []
        for h in self.hazards:
            if h.state == Hazard.PASSED:
                self.noticed.pop(h.id, None)  # a new pass gets a new draw
            if h.frequency <= 0 or gaze is None:
                continue
            gx, gy, gyaw = gaze
            bearing = math.atan2(h.position.y - gy, h.position.x - gx)
            off = abs(math.atan2(math.sin(bearing - gyaw), math.cos(bearing - gyaw)))
            if off < self.view_half_angle and math.hypot(h.position.x - gx, h.position.y - gy) < 10.0:
                if h.id not in self.noticed:
                    self.noticed[h.id] = random.random() < self.detect_prob
                    self.get_logger().info(f'Simulated user {"NOTICES" if self.noticed[h.id] else "misses"} '
                                           f'hazard {h.id} ({h.name})')
                if self.noticed[h.id]:
                    seen.append(h.frequency)
        self.active = seen

    def stream(self):
        rng = np.random.default_rng()
        drift = np.zeros(len(LABELS))
        alpha_phase = 0.0
        sample = 0
        start = pylsl.local_clock()
        while rclpy.ok():
            # pace the output at 256 Hz
            target = start + (sample + CHUNK) / FS
            delay = target - pylsl.local_clock()
            if delay > 0:
                time.sleep(delay)
            t = (sample + np.arange(CHUNK)) / FS
            chunk = np.zeros((CHUNK, len(LABELS)))
            for i in range(CHUNK):
                drift = 0.97 * drift + rng.normal(0, 1, len(LABELS))   # slow, 1/f-like background
                chunk[i] = drift * self.noise_uv * 0.25 + rng.normal(0, self.noise_uv * 0.5, len(LABELS))
            alpha_amp = self.noise_uv * (0.6 + 0.4 * math.sin(sample / FS * 0.7))
            alpha = alpha_amp * np.sin(2 * np.pi * 10.0 * t + alpha_phase)
            chunk[:, 4] += alpha
            for f in list(self.active):
                chunk[:, 4] += self.amplitude * np.sin(2 * np.pi * f * t)
            stamp = start + (sample + CHUNK - 1) / FS  # LSL wants the time of the chunk's last sample
            self.outlet.push_chunk(chunk.tolist(), stamp)
            sample += CHUNK


def main():
    rclpy.init()
    node = MuseEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

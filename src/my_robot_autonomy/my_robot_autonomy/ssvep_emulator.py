"""A stand-in for the SSVEP decoder's verdicts, for desktop and sim testing.

Publishes /probe_results - the same topic the real decoder (eeg_decoder) publishes - so swapping
it for the real Muse pipeline needs no other change. Also publishes a minimal /eeg_status.

  mode toggle  nothing is registered until you press 'e' in walker_keys (or publish
               std_msgs/Empty on /ssvep_emulator/toggle): the nearest probed hazard is registered.
  mode model   each probed hazard is registered after a random 2-4 s with probability p_detect;
               false "registered" verdicts arrive at p_false per second per probed hazard.
  eeg_valid    false: the EEG signal counts as unreliable (signal_ok false, nothing registers).

(muse_emulator is the other emulator: it makes a raw EEG stream for the real decoder to analyse.)
"""
import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from walker_interfaces.msg import Hazard, HazardArray, ProbeResult


class SsvepEmulator(Node):
    def __init__(self):
        super().__init__('ssvep_emulator')
        p = self.declare_parameter
        self.mode = p('mode', 'model').value
        self.p_detect = p('p_detect', 0.8).value
        self.p_false = p('p_false', 0.01).value
        self.eeg_valid = p('eeg_valid', True).value
        self.delay = (p('min_delay', 2.0).value, p('max_delay', 4.0).value)
        seed = p('seed', 0).value
        self.rng = random.Random(seed if seed else None)
        self.hazards = []
        self.plans = {}       # (hazard id, frequency) -> time it will be registered, or None
        self.toggled = set()
        self.result_pub = self.create_publisher(ProbeResult, 'probe_results', 50)
        self.status_pub = self.create_publisher(String, 'eeg_status', 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(Empty, 'ssvep_emulator/toggle', self.on_toggle, 10)
        self.create_timer(0.5, self.update)
        self.get_logger().info(f'SSVEP emulator: mode {self.mode}, p_detect {self.p_detect}, '
                               f'eeg_valid {self.eeg_valid}')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_toggle(self, _):
        probed = [h for h in self.hazards if h.frequency > 0 and h.state in (Hazard.PROBING, Hazard.MISSED)]
        if probed:
            h = min(probed, key=lambda h: h.path_distance)
            self.toggled.add((h.id, round(h.frequency, 2)))
            self.get_logger().info(f'Toggle: hazard {h.id} ({h.name}) registered')

    def update(self):
        now = self.now()
        self.status_pub.publish(String(data=json.dumps({
            'stream': 'OK' if self.eeg_valid else 'POOR', 'quality': 'OK' if self.eeg_valid else 'BAD',
            'source': f'ssvep_emulator ({self.mode})', 'check': {'state': 'pass' if self.eeg_valid else 'fail'}})))
        for h in self.hazards:
            if h.frequency <= 0 or h.state not in (Hazard.PROBING, Hazard.MISSED):
                continue
            key = (h.id, round(h.frequency, 2))
            if key not in self.plans:
                noticed = self.rng.random() < self.p_detect
                self.plans[key] = now + self.rng.uniform(*self.delay) if noticed else None
            if not self.eeg_valid:
                present = False
            elif self.mode == 'toggle':
                present = key in self.toggled
            else:
                at = self.plans[key]
                present = (at is not None and now >= at) or self.rng.random() < self.p_false * 0.5
            r = ProbeResult()
            r.stamp = self.get_clock().now().to_msg()
            r.hazard_id, r.frequency = h.id, h.frequency
            r.score, r.present, r.signal_ok = (5.0 if present else 1.2), present, self.eeg_valid
            self.result_pub.publish(r)


def main():
    rclpy.init()
    node = SsvepEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Records a VR session to ~/eye2steering_ws/sessions/<date-time>/:

  events.jsonl   one line per event: hazard state changes, EEG results, walker status
                 changes, operator actions (start/stop walking, flicker, commands)
  path.csv       the walker's position, speed cap and state, twice a second
  hazards.json   every hazard's final record, written when the session ends
"""
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from walker_interfaces.msg import Hazard, HazardArray, ProbeResult

STATE_NAMES = {Hazard.WAITING: 'waiting', Hazard.PROBING: 'probing', Hazard.REGISTERED: 'registered',
               Hazard.MISSED: 'missed', Hazard.PASSED: 'passed'}


class SessionLogger(Node):
    def __init__(self):
        super().__init__('session_logger')
        base = os.path.expanduser(self.declare_parameter('sessions_dir', '~/eye2steering_ws/sessions').value)
        self.dir = os.path.join(base, datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
        os.makedirs(self.dir, exist_ok=True)
        self.events = open(os.path.join(self.dir, 'events.jsonl'), 'a', buffering=1)
        self.path = open(os.path.join(self.dir, 'path.csv'), 'a', buffering=1)
        self.path.write('time,x,y,yaw,state,speed_cap_percent\n')

        self.states = {}
        self.records = {}
        self.status = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(HazardArray, 'hazards', self.on_hazards, 10)
        self.create_subscription(ProbeResult, 'probe_results', self.on_result, 50)
        self.create_subscription(String, 'walker_status', self.on_status, latched)
        self.create_subscription(Bool, 'walker_enabled', lambda m: self.log('walker_enabled', value=m.data), latched)
        self.create_subscription(Bool, 'flicker_enabled', lambda m: self.log('flicker_enabled', value=m.data), latched)
        self.create_subscription(String, 'operator_command', lambda m: self.log('operator_command', value=m.data), 10)
        self.create_timer(0.5, self.log_path)
        self.get_logger().info(f'Recording this session to {self.dir}')

    def sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def log(self, kind, **data):
        self.events.write(json.dumps({'time': round(self.sim_time(), 2), 'wall_time': time.time(),
                                      'event': kind, **data}) + '\n')

    def on_hazards(self, msg):
        for h in msg.hazards:
            record = self.records.setdefault(h.id, {'id': h.id, 'name': h.name, 'x': h.position.x,
                                                    'y': h.position.y, 'history': []})
            if self.states.get(h.id) != h.state:
                self.states[h.id] = h.state
                entry = {'time': round(self.sim_time(), 2), 'state': STATE_NAMES[h.state],
                         'path_distance': round(h.path_distance, 2), 'frequency': round(h.frequency, 2)}
                record['history'].append(entry)
                self.log('hazard_state', hazard_id=h.id, name=h.name, **entry)

    def on_result(self, msg):
        self.log('probe_result', hazard_id=msg.hazard_id, frequency=round(msg.frequency, 2),
                 score=round(msg.score, 2), present=msg.present, signal_ok=msg.signal_ok)

    def on_status(self, msg):
        status = json.loads(msg.data)
        key = (status.get('state'), status.get('speed_cap_percent'), status.get('extra_clearance'))
        if key != (self.status.get('state'), self.status.get('speed_cap_percent'), self.status.get('extra_clearance')):
            self.log('walker_status', **status)
        self.status = status

    def log_path(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.path.write(f'{self.sim_time():.2f},{t.transform.translation.x:.3f},{t.transform.translation.y:.3f},'
                        f'{yaw:.3f},{self.status.get("state", "")},{self.status.get("speed_cap_percent", "")}\n')

    def close(self):
        with open(os.path.join(self.dir, 'hazards.json'), 'w') as f:
            json.dump(list(self.records.values()), f, indent=2)
        self.events.close()
        self.path.close()


def main():
    rclpy.init()
    node = SessionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

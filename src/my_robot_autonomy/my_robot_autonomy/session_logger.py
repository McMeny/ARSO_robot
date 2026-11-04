"""Records a VR session to ~/eye2steering_ws/sessions/<date-time>/:

  events.jsonl   one line per event: hazard state changes, EEG results, walker status
                 changes, operator actions (start/stop walking, flicker, commands)
  path.csv       the walker's pose, the head pose, grip state, steering (authority, desired yaw,
                 turning in place, blocked, caution) and speed cap, 10 times a second
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

from geometry_msgs.msg import PoseStamped

from walker_interfaces.msg import GripState, Hazard, HazardArray, ProbeResult, SteeringStatus

GRIP_NAMES = {GripState.STOP: 'STOP', GripState.WALK: 'WALK', GripState.STEER: 'STEER'}

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
        self.path.write('time,x,y,yaw,state,speed_cap_percent,head_x,head_y,head_z,head_yaw,grip,grip_left,'
                        'grip_right,authority,desired_yaw,turning_in_place,blocked,caution\n')
        self.head = None
        self.grip = None
        self.steering = None
        self.last_steering = None

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
        self.create_subscription(PoseStamped, 'head_pose', lambda m: setattr(self, 'head', m), 10)
        self.create_subscription(GripState, 'walker/grip_state', self.on_grip, 10)
        self.create_subscription(SteeringStatus, 'walker/steering_status', self.on_steering, 10)
        self.create_timer(0.1, self.log_path)
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

    def on_grip(self, msg):
        if self.grip is None or msg.state != self.grip.state:
            self.log('grip_state', state=GRIP_NAMES.get(msg.state, msg.state),
                     left=round(msg.left, 2), right=round(msg.right, 2))
        self.grip = msg

    def on_steering(self, msg):
        key = (msg.turning_in_place, msg.blocked, msg.caution, msg.hint, msg.notice)
        if key != self.last_steering:
            self.log('steering_status', turning_in_place=msg.turning_in_place, blocked=msg.blocked,
                     caution=msg.caution, hint=msg.hint, notice=msg.notice, authority=round(msg.authority, 2))
            self.last_steering = key
        self.steering = msg

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
        row = [f'{self.sim_time():.2f}', f'{t.transform.translation.x:.3f}', f'{t.transform.translation.y:.3f}',
               f'{yaw:.3f}', str(self.status.get('state', '')), str(self.status.get('speed_cap_percent', ''))]
        if self.head is not None:
            hp, hq = self.head.pose.position, self.head.pose.orientation
            hyaw = math.atan2(2 * (hq.w * hq.z + hq.x * hq.y), 1 - 2 * (hq.y * hq.y + hq.z * hq.z))
            row += [f'{hp.x:.3f}', f'{hp.y:.3f}', f'{hp.z:.3f}', f'{hyaw:.3f}']
        else:
            row += [''] * 4
        g = self.grip
        row += [GRIP_NAMES.get(g.state, ''), f'{g.left:.2f}', f'{g.right:.2f}'] if g else [''] * 3
        st = self.steering
        row += ([f'{st.authority:.2f}', f'{st.desired_yaw:.3f}', str(int(st.turning_in_place)), str(int(st.blocked)),
                 str(int(st.caution))] if st else [''] * 5)
        self.path.write(','.join(row) + '\n')

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

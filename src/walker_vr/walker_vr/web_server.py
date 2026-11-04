"""The bridge between ROS and the browser pages (headset, spectator, operator).

One port (default 8443) serves:
  /                 the headset app (WebXR)          index.html
  /spectator.html   the participant's view on a PC   spectator.html
  /operator.html    start/stop, flicker, checks      operator.html
  /world.json       the store's objects, from the same world file Gazebo uses
  /ws               a WebSocket carrying JSON messages both ways

HTTPS if ~/.walker_vr/cert.pem exists (run: ros2 run walker_vr make_certs) - the Quest
needs it. Without it, plain HTTP, which still works for http://localhost on this PC.

ROS -> browsers: walker_pose (30 Hz), hazards, trajectory, walker_status, eeg_status,
                 test_stimulus, head_pose (for the spectator), command (e.g. recentre)
browsers -> ROS: head_pose -> /head_pose, headset_status -> /headset_status,
                 walker_enabled -> /walker_enabled, flicker_enabled -> /flicker_enabled,
                 command -> /operator_command
"""
import json
import math
import os
import ssl
import threading

import rclpy
import tornado.ioloop
import tornado.web
import tornado.websocket
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException

from my_robot_autonomy.world_model import load_world
from walker_interfaces.msg import HazardArray, WalkerTrajectory

CERT_DIR = os.path.expanduser('~/.walker_vr')
STATE_NAMES = ['waiting', 'probing', 'registered', 'missed', 'passed']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Bridge(Node):
    def __init__(self, broadcast):
        super().__init__('walker_web_server')
        p = self.declare_parameter
        self.port = p('port', 8443).value
        self.world_file = p('world_file', '').value
        self.broadcast = broadcast
        self.latest = {}  # message type -> last message, sent to pages that connect later

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.head_pub = self.create_publisher(PoseStamped, 'head_pose', 10)
        self.headset_pub = self.create_publisher(String, 'headset_status', 10)
        self.enabled_pub = self.create_publisher(Bool, 'walker_enabled', latched)
        self.flicker_pub = self.create_publisher(Bool, 'flicker_enabled', latched)
        self.command_pub = self.create_publisher(String, 'operator_command', 10)

        self.create_subscription(HazardArray, 'hazards', self.on_hazards, 10)
        self.create_subscription(WalkerTrajectory, 'walker_trajectory', self.on_trajectory, 10)
        self.create_subscription(String, 'walker_status', lambda m: self.send('walker_status', json.loads(m.data)), latched)
        self.create_subscription(String, 'eeg_status', lambda m: self.send('eeg_status', json.loads(m.data)), 10)
        self.create_subscription(String, 'test_stimulus', lambda m: self.send('test_stimulus', json.loads(m.data)), 10)
        self.create_subscription(Bool, 'walker_enabled', lambda m: self.send('walker_enabled', {'value': m.data}), latched)
        self.create_subscription(Bool, 'flicker_enabled', lambda m: self.send('flicker_enabled', {'value': m.data}), latched)

        # The walker starts stopped; flicker starts on. (The operator page changes both.)
        self.enabled_pub.publish(Bool(data=p('start_enabled', False).value))
        self.flicker_pub.publish(Bool(data=True))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1 / 30, self.send_walker_pose)

    def send(self, kind, data, remember=True):
        message = json.dumps({'type': kind, **data})
        if remember:
            self.latest[kind] = message
        self.broadcast(message)

    def send_walker_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        self.send('walker_pose', {'x': t.transform.translation.x, 'y': t.transform.translation.y,
                                  'yaw': yaw_of(t.transform.rotation)})

    def on_hazards(self, msg):
        self.send('hazards', {'hazards': [{
            'id': h.id, 'name': h.name, 'kind': h.kind, 'x': h.position.x, 'y': h.position.y, 'z': h.position.z,
            'size_x': h.size_x, 'size_y': h.size_y, 'height': h.height, 'yaw': h.yaw,
            'path_distance': round(h.path_distance, 2), 'lateral': round(h.lateral, 2),
            'relevant': h.relevant, 'state': STATE_NAMES[h.state], 'frequency': h.frequency,
            'contrast': h.contrast, 'confidence': round(h.confidence, 2)} for h in msg.hazards]})

    def on_trajectory(self, msg):
        pts = lambda arr: [[round(p.x, 3), round(p.y, 3)] for p in arr]
        self.send('trajectory', {'centre': pts(msg.centre), 'left': pts(msg.left_edge), 'right': pts(msg.right_edge),
                                 'extra_clearance': msg.extra_clearance,
                                 'machine_authority': msg.machine_authority})

    def from_browser(self, data):
        kind = data.get('type')
        if kind == 'head_pose':
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = data['x'], data['y'], data['z']
            # yaw about z, then pitch about the new y (nose down = +pitch)
            cy, sy = math.cos(data['yaw'] / 2), math.sin(data['yaw'] / 2)
            cp, sp = math.cos(data['pitch'] / 2), math.sin(data['pitch'] / 2)
            pose.pose.orientation.w, pose.pose.orientation.x = cy * cp, -sy * sp
            pose.pose.orientation.y, pose.pose.orientation.z = cy * sp, sy * cp
            self.head_pub.publish(pose)
            self.send('head_pose', data, remember=False)
        elif kind == 'headset_status':
            self.headset_pub.publish(String(data=json.dumps(data)))
            self.send('headset_status', data)
        elif kind == 'walker_enabled':
            self.enabled_pub.publish(Bool(data=bool(data['value'])))
        elif kind == 'flicker_enabled':
            self.flicker_pub.publish(Bool(data=bool(data['value'])))
        elif kind == 'command':
            name = str(data.get('name', ''))
            self.command_pub.publish(String(data=name))
            self.send('command', {'name': name}, remember=False)
            self.get_logger().info(f'Operator: {name}')


class Socket(tornado.websocket.WebSocketHandler):
    clients = set()
    bridge = None

    def check_origin(self, origin):
        return True

    def open(self):
        Socket.clients.add(self)
        for message in list(Socket.bridge.latest.values()):
            self.write_message(message)

    def on_close(self):
        Socket.clients.discard(self)

    def on_message(self, message):
        try:
            Socket.bridge.from_browser(json.loads(message))
        except (ValueError, KeyError, TypeError) as e:
            Socket.bridge.get_logger().warn(f'Bad message from a browser: {e}')


class WorldJson(tornado.web.RequestHandler):
    objects = []

    def get(self):
        self.set_header('Content-Type', 'application/json')
        self.write(json.dumps({'objects': WorldJson.objects}))


class NoCache(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-store')


def main():
    rclpy.init()
    loop = tornado.ioloop.IOLoop.current()

    def broadcast(message):
        def send_all():
            for c in list(Socket.clients):
                try:
                    c.write_message(message)
                except tornado.websocket.WebSocketClosedError:
                    Socket.clients.discard(c)
        loop.add_callback(send_all)

    bridge = Bridge(broadcast)
    Socket.bridge = bridge
    if bridge.world_file:
        WorldJson.objects = [{'name': o.name, 'kind': o.kind, 'x': o.x, 'y': o.y, 'z': o.z, 'yaw': o.yaw,
                              'size_x': o.size_x, 'size_y': o.size_y, 'height': o.height, 'colour': o.colour}
                             for o in load_world(bridge.world_file)]

    web_dir = os.path.join(get_package_share_directory('walker_vr'), 'web')
    app = tornado.web.Application([
        (r'/ws', Socket),
        (r'/world.json', WorldJson),
        (r'/(.*)', NoCache, {'path': web_dir, 'default_filename': 'index.html'}),
    ])

    cert, key = os.path.join(CERT_DIR, 'cert.pem'), os.path.join(CERT_DIR, 'key.pem')
    if os.path.exists(cert) and os.path.exists(key):
        ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ctx.load_cert_chain(cert, key)
        app.listen(bridge.port, ssl_options=ctx)
        bridge.get_logger().info(f'Serving https://<this PC>:{bridge.port}  (headset: /  spectator: /spectator.html  '
                                 f'operator: /operator.html)')
    else:
        app.listen(bridge.port)
        bridge.get_logger().warn(f'No certificate in {CERT_DIR} - serving plain http://localhost:{bridge.port}. '
                                 f'The Quest needs https: run "ros2 run walker_vr make_certs" and restart.')

    executor = MultiThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        loop.start()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    bridge.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

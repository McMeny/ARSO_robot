"""The walker's short-term trajectory for the floor display: a centre line and two edge
lines marking its footprint over the next horizon seconds (at least min_length metres,
so it stays visible at walking pace).

Built from Nav2's plan, starting at the walker. The edges widen by the extra clearance
walker_brain is giving a missed hazard, and carry the machine's current authority (the
headset draws them more solid as the machine takes more control).

/plan + /walker_status + /machine_authority  ->  /walker_trajectory (walker_interfaces/WalkerTrajectory)
                                                 /trajectory_markers (RViz)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from walker_interfaces.msg import WalkerTrajectory


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TrajectoryOverlay(Node):
    def __init__(self):
        super().__init__('trajectory_overlay')
        p = self.declare_parameter
        self.horizon = p('horizon', 2.0).value            # seconds of travel shown
        self.min_length = p('min_length', 1.5).value      # metres, so it's visible at walking pace
        self.max_speed = p('max_speed', 0.26).value       # the walker's normal pace (m/s)
        self.half_width = p('half_width', 0.25).value     # half the walker's width
        self.front = p('front_offset', 0.46).value        # the line starts at the walker's front

        self.path = None
        self.extra = 0.0
        self.cap = 100.0
        self.authority = 1.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(WalkerTrajectory, 'walker_trajectory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'trajectory_markers', 10)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(String, 'walker_status', self.on_status, latched)
        self.create_subscription(Float32, 'machine_authority', lambda m: setattr(self, 'authority', m.data), 10)
        self.create_timer(0.1, self.update)

    def on_plan(self, msg):
        if msg.poses:
            self.path = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])

    def on_status(self, msg):
        status = json.loads(msg.data)
        self.extra = status.get('extra_clearance', 0.0)
        self.cap = status.get('speed_cap_percent', 100.0) if status.get('enabled', True) else 0.0

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        x, y = tf.transform.translation.x, tf.transform.translation.y
        yaw = yaw_of(tf.transform.rotation)
        length = max(self.min_length, self.max_speed * self.cap / 100.0 * self.horizon)

        # the path: from the walker's front, along Nav2's plan (or straight ahead if there's none)
        fx, fy = x + self.front * math.cos(yaw), y + self.front * math.sin(yaw)
        pts = [np.array([fx, fy])]
        if self.path is not None and len(self.path) >= 2:
            start = int(np.argmin(np.hypot(self.path[:, 0] - fx, self.path[:, 1] - fy)))
            pts += list(self.path[start + 1:])
        if len(pts) < 2:
            pts.append(np.array([fx + length * math.cos(yaw), fy + length * math.sin(yaw)]))

        # resample to 0.05 m steps up to `length`
        centre = [pts[0]]
        travelled = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            seg = float(np.linalg.norm(b - a))
            if seg < 1e-6:
                continue
            n = max(int(seg / 0.05), 1)
            for j in range(1, n + 1):
                travelled += seg / n
                centre.append(a + (b - a) * j / n)
                if travelled >= length:
                    break
            if travelled >= length:
                break
        if travelled < length:  # plan ended early - carry on straight
            last = centre[-1]
            d = centre[-1] - centre[-2] if len(centre) > 1 else np.array([math.cos(yaw), math.sin(yaw)])
            d = d / (np.linalg.norm(d) or 1.0)
            steps = int((length - travelled) / 0.05)
            centre += [last + d * 0.05 * (j + 1) for j in range(steps)]

        width = self.half_width + self.extra
        left, right = [], []
        for i, c in enumerate(centre):
            a = centre[max(i - 1, 0)]
            b = centre[min(i + 1, len(centre) - 1)]
            d = b - a
            n = np.array([-d[1], d[0]]) / (np.linalg.norm(d) or 1.0)
            left.append(c + n * width)
            right.append(c - n * width)

        msg = WalkerTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        to_pt = lambda v: Point(x=float(v[0]), y=float(v[1]), z=0.01)
        msg.centre = [to_pt(c) for c in centre]
        msg.left_edge = [to_pt(c) for c in left]
        msg.right_edge = [to_pt(c) for c in right]
        msg.extra_clearance = float(self.extra)
        msg.machine_authority = float(self.authority)
        self.pub.publish(msg)
        self.publish_markers(msg)

    def publish_markers(self, msg):
        markers = []
        for i, (pts, width) in enumerate(((msg.centre, 0.02), (msg.left_edge, 0.04), (msg.right_edge, 0.04))):
            m = Marker()
            m.header = msg.header
            m.ns, m.id, m.type, m.action = 'trajectory', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b = (1.0, 1.0, 1.0) if i == 0 else (0.3, 0.9, 1.0)
            m.color.a = 0.4 + 0.6 * msg.machine_authority  # more solid as the machine takes over
            m.points = list(pts)
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = TrajectoryOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""The tentacle fan: candidate arcs from hard left through straight to hard right, each
checked against the lidar to see how far it is clear.

/scan in  ->  /tentacles (walker_interfaces/Tentacles) and /tentacle_markers (RViz)

Each arc starts at the walker's front edge and is checked over `length` metres, in a
corridor `half_width` either side (a bit wider than the walker). The free length of the
straight arc is the walker's "clearance" - the risk measure used for shared steering.
forward_goal uses the fan to see which ways forward are open.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from walker_interfaces.msg import Tentacles


def arc_coordinates(k, x, y):
    """For points (x, y) in base_footprint: distance along an arc of curvature k that
    starts at the origin heading +x (s), and sideways distance from that arc (lateral)."""
    if abs(k) < 1e-3:
        s = np.where(x >= 0, x, np.inf)
        return s, np.abs(y)
    radius = 1.0 / k                    # centre of the turn is at (0, radius)
    dx, dy = x, y - radius
    lateral = np.abs(np.hypot(dx, dy) - abs(radius))
    swept = np.arctan2(dx, -dy) if k > 0 else np.arctan2(dx, dy)
    swept = np.where(swept < 0, np.inf, swept)  # behind the robot
    return abs(radius) * swept, lateral


def arc_point(k, s, side=0.0):
    """Point s metres along an arc of curvature k, side metres to its left."""
    if abs(k) < 1e-3:
        return s, side
    a = s * k
    return math.sin(a) / k - side * math.sin(a), (1 - math.cos(a)) / k + side * math.cos(a)


class TentacleFan(Node):
    def __init__(self):
        super().__init__('tentacles')
        self.count = self.declare_parameter('count', 9).value
        self.max_curvature = self.declare_parameter('max_curvature', 1.2).value
        self.length = self.declare_parameter('length', 3.0).value
        self.half_width = self.declare_parameter('half_width', 0.3).value
        self.front = self.declare_parameter('front_offset', 0.46).value
        self.curvatures = np.linspace(self.max_curvature, -self.max_curvature, self.count)  # left -> right

        self.laser_offset = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Tentacles, 'tentacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tentacle_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def arc_length(self, k):
        """Don't let a tight arc curl round more than ~160 degrees."""
        if abs(k) < 1e-3:
            return self.length
        return min(self.length, 0.9 * math.pi / abs(k) - self.front)

    def on_scan(self, scan):
        if self.laser_offset is None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', scan.header.frame_id, rclpy.time.Time())
            except TransformException:
                return
            self.laser_offset = (t.transform.translation.x, t.transform.translation.y)

        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        x = self.laser_offset[0] + ranges[valid] * np.cos(angles[valid])
        y = self.laser_offset[1] + ranges[valid] * np.sin(angles[valid])

        msg = Tentacles()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = 'base_footprint'
        msg.length, msg.half_width = float(self.length), float(self.half_width)
        for k in self.curvatures:
            length = self.arc_length(k)
            s, lateral = arc_coordinates(k, x, y)
            ahead = s - self.front
            hit = (ahead > -0.05) & (ahead < length) & (lateral < self.half_width)
            free = float(max(np.min(ahead[hit]), 0.0)) if np.any(hit) else length
            msg.curvatures.append(float(k))
            msg.free_lengths.append(free)
        msg.clearance = msg.free_lengths[self.count // 2]
        self.pub.publish(msg)
        self.publish_markers(msg)

    def publish_markers(self, msg):
        markers = []
        for i, (k, free) in enumerate(zip(msg.curvatures, msg.free_lengths)):
            length = self.arc_length(k)
            m = Marker()
            m.header = msg.header
            m.ns, m.id, m.type, m.action = 'tentacles', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.025
            clear = free / length if length > 0 else 0.0
            # green = clear all the way, red = blocked close by
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0 - clear, clear, 0.2, 0.8
            steps = 12
            m.points = [Point(x=px, y=py, z=0.03) for px, py in
                        (arc_point(k, self.front + free * j / steps) for j in range(steps + 1))]
            markers.append(m)
        self.marker_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = TentacleFan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

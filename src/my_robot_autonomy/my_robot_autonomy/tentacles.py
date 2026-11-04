"""The tentacle fan: candidate arcs from hard left through straight to hard right, each
checked against the lidar to see how far it is clear.

/scan in  ->  /tentacles (walker_interfaces/Tentacles) and /tentacle_markers (RViz)

Each arc starts at the walker's front edge and is checked over `length` metres, in a
corridor `half_width` either side (a bit wider than the walker). On a curve the corridor is
widened to cover the walker's outer front corner, which swings wider than the centre line,
and obstacles alongside the body count too. Lidar points on a MISSED hazard (walker_brain's
missed_hazards) get the extra clearance on top - the wider berth is for that hazard only. The free length of the
straight arc is the walker's "clearance" - the risk measure used for shared steering.
tentacle_driver picks the arc to drive along from this fan.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from walker_interfaces.msg import HazardArray, Tentacles


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
        self.count = self.declare_parameter('count', 15).value
        self.max_curvature = self.declare_parameter('max_curvature', 1.2).value
        self.length = self.declare_parameter('length', 3.0).value
        self.half_width = self.declare_parameter('half_width', 0.35).value   # body 0.25 + 10 cm margin
        self.front = self.declare_parameter('front_offset', 0.46).value
        self.body_half_width = self.declare_parameter('body_half_width', 0.25).value
        self.min_usable = self.declare_parameter('min_usable_free_length', 0.55).value  # = driver's stop + margin
        self.curvatures = np.linspace(self.max_curvature, -self.max_curvature, self.count)  # left -> right

        self.laser_offset = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Tentacles, 'tentacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'tentacle_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.extra = 0.0
        self.missed_ids = set()
        self.hazards = []
        self.create_subscription(String, 'walker_status', self.on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)

    def on_status(self, msg):
        status = json.loads(msg.data)
        self.extra = float(status.get('extra_clearance', 0.0))
        self.missed_ids = set(status.get('missed_hazards', []))

    def corridor(self, k):
        """Half-width to check around an arc: covers the outer front corner on a curve."""
        if abs(k) < 1e-3:
            return self.half_width
        r = 1.0 / abs(k)
        outer = math.hypot(r + self.body_half_width, self.front) - r
        return max(self.half_width, outer + (self.half_width - self.body_half_width))

    def on_missed_hazard(self, x, y):
        """Which lidar points (base_footprint) lie on a missed hazard - they get the extra berth."""
        mask = np.zeros(len(x), dtype=bool)
        missed = [h for h in self.hazards if h.id in self.missed_ids]
        if not missed or self.extra <= 0:
            return mask
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return mask
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        mx = t.transform.translation.x + math.cos(yaw) * x - math.sin(yaw) * y
        my = t.transform.translation.y + math.sin(yaw) * x + math.cos(yaw) * y
        for h in missed:
            dx, dy = mx - h.position.x, my - h.position.y
            if h.kind == 'cylinder':
                gap = np.hypot(dx, dy) - h.size_x / 2
            else:
                c, s = math.cos(-h.yaw), math.sin(-h.yaw)
                lx, ly = c * dx - s * dy, s * dx + c * dy
                gap = np.hypot(np.maximum(np.abs(lx) - h.size_x / 2, 0), np.maximum(np.abs(ly) - h.size_y / 2, 0))
            mask |= gap < 0.15
        return mask

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
        on_missed = self.on_missed_hazard(x, y)
        result = self.check_arcs(x, y, np.where(on_missed, self.extra, 0.0))
        # the wider berth only where it fits - if it would block every arc, fall back to the normal margin
        if np.any(on_missed) and max(f for f, _ in result) < self.min_usable:
            result = self.check_arcs(x, y, np.zeros(len(x)))
        for k, (free, gap) in zip(self.curvatures, result):
            msg.curvatures.append(float(k))
            msg.free_lengths.append(free)
            msg.side_gaps.append(gap)
        msg.clearance = msg.free_lengths[self.count // 2]
        self.pub.publish(msg)
        self.publish_markers(msg)

    def check_arcs(self, x, y, extra):
        """(free length, side gap) for every arc, with `extra` clearance per lidar point."""
        out = []
        for k in self.curvatures:
            length = self.arc_length(k)
            s, lateral = arc_coordinates(k, x, y)
            ahead = s - self.front
            width = self.corridor(k) + extra
            # straight on, only what's ahead of the front matters; on a curve the body's side
            # sweeps too, so obstacles alongside it count
            start = -0.05 if abs(k) < 0.1 else -self.front
            hit = (ahead > start) & (ahead < length) & (lateral < width)
            free = float(max(np.min(ahead[hit]), 0.0)) if np.any(hit) else length
            # how close things come to the walker's side along the free part (for keeping to the middle)
            along = (ahead > start) & (ahead < free) & (lateral < 1.5)
            gap = float(np.min(lateral[along]) - self.body_half_width) if np.any(along) else 1.25
            out.append((free, max(gap, 0.0)))
        return out

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

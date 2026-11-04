"""Watches the lane the robot is about to drive through, in the lidar scan.

/scan + /odometry/filtered in  ->  /obstacle_ahead (Bool) and
                                   /obstacle_distance (Float32, metres along the lane, inf if clear)

The lane is a bit wider than the robot and starts at its front edge. It follows the
robot's current motion: straight when driving straight, curved along the turn when
turning (curvature = turn speed / forward speed, from the EKF). So shelves beside the
robot in an aisle don't count, but whatever it is turning towards does.

For RViz it also draws the lane on /walker_markers, coloured by /walker_state
(green = cruise, yellow = obstacle + human aware, orange = obstacle + human not aware),
with a red ball on the closest obstacle point.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

STATE_COLOURS = {
    'CRUISE': (0.2, 0.9, 0.3),
    'NORMAL_AVOID': (1.0, 0.85, 0.1),
    'CAREFUL_AVOID': (1.0, 0.4, 0.1),
}
MAX_CURVATURE = 1.0  # tightest lane bend, 1/m (a 1 m radius)
TURN_DEADBAND = 0.15  # rad/s - ignore Nav2's small steering corrections, keep the lane straight
CRUISE_SPEED = 0.25   # m/s - bend measured against normal speed, so slowing down doesn't curl the lane


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # how far ahead to look (along the lane), and half the lane width (robot is ~0.5 m wide)
        self.detect_distance = self.declare_parameter('detect_distance', 2.0).value
        self.half_width = self.declare_parameter('half_width', 0.4).value
        # where the lane starts - the robot's front edge, metres ahead of base_footprint
        self.front = self.declare_parameter('front_offset', 0.46).value

        self.state = 'CRUISE'
        self.curvature = 0.0       # smoothed, + = turning left
        self.laser_offset = None   # (x, y) of the lidar in base_footprint, from TF

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ahead_pub = self.create_publisher(Bool, 'obstacle_ahead', 10)
        self.distance_pub = self.create_publisher(Float32, 'obstacle_distance', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'walker_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/filtered', self.on_odom, 10)
        self.create_subscription(String, 'walker_state', lambda m: setattr(self, 'state', m.data), 10)

    def on_odom(self, odom):
        v = odom.twist.twist.linear.x
        w = odom.twist.twist.angular.z
        if abs(w) < TURN_DEADBAND:
            w = 0.0
        # curvature = turn speed / forward speed, but never divide by less than cruise
        # speed - at 30 % speed (or on the spot) a small turn would otherwise curl the lane
        k = max(-MAX_CURVATURE, min(MAX_CURVATURE, w / max(abs(v), CRUISE_SPEED)))
        if v < -0.02:
            k = -k  # reversing - the lane is drawn forwards, so mirror the turn
        self.curvature = 0.9 * self.curvature + 0.1 * k  # smooth out jitter (~0.3 s at 30 Hz)

    def lane_coordinates(self, x, y):
        """For points (x, y) in base_footprint: distance along the robot's path (s) and
        sideways distance from it (lateral). Path = an arc of the current curvature."""
        k = self.curvature
        if abs(k) < 1e-3:
            return x, np.abs(y)
        radius = 1.0 / k                    # centre of the turn is at (0, radius)
        dx, dy = x, y - radius
        lateral = np.abs(np.hypot(dx, dy) - abs(radius))
        # angle swept around the centre from the robot to the point, in the driving direction
        swept = np.arctan2(dx, -dy) if k > 0 else np.arctan2(dx, dy)
        swept = np.where(swept < 0, np.inf, swept)  # behind the robot - not in the lane
        return abs(radius) * swept, lateral

    def lane_length(self):
        """detect_distance, but on a tight turn stop before the lane curls round more
        than ~160 degrees (it would loop back towards the robot)."""
        if abs(self.curvature) < 1e-3:
            return self.detect_distance
        return min(self.detect_distance, 0.9 * math.pi / abs(self.curvature) - self.front)

    def lane_point(self, s, side):
        """Point s metres along the path, side metres to the left of it (base_footprint)."""
        k = self.curvature
        if abs(k) < 1e-3:
            return s, side
        a = s * k
        return (math.sin(a) / k - side * math.sin(a),
                (1 - math.cos(a)) / k + side * math.cos(a))

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
        # lidar points in base_footprint (the lidar points the same way as the robot)
        x = self.laser_offset[0] + ranges[valid] * np.cos(angles[valid])
        y = self.laser_offset[1] + ranges[valid] * np.sin(angles[valid])

        s, lateral = self.lane_coordinates(x, y)
        ahead = s - self.front
        in_lane = (ahead > -0.05) & (ahead < self.lane_length()) & (lateral < self.half_width)

        closest, hit = math.inf, None
        if np.any(in_lane):
            i = np.argmin(np.where(in_lane, ahead, np.inf))
            closest, hit = max(float(ahead[i]), 0.0), (float(x[i]), float(y[i]))

        self.ahead_pub.publish(Bool(data=closest < math.inf))
        self.distance_pub.publish(Float32(data=closest))
        self.publish_markers(scan.header.stamp, hit)

    def publish_markers(self, stamp, hit):
        r, g, b = STATE_COLOURS.get(self.state, (0.7, 0.7, 0.7))
        w = self.half_width

        def marker(ns, kind):
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, 0, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b = r, g, b
            return m

        # the lane's two edges, sampled along its length, on the floor
        length = self.lane_length()
        steps = [self.front + length * i / 20 for i in range(21)]
        left = [Point(x=px, y=py, z=0.02) for px, py in (self.lane_point(s, w) for s in steps)]
        right = [Point(x=px, y=py, z=0.02) for px, py in (self.lane_point(s, -w) for s in steps)]

        outline = marker('detect_lane', Marker.LINE_STRIP)
        outline.scale.x = 0.04
        outline.color.a = 1.0
        outline.points = left + right[::-1] + [left[0]]

        # see-through fill - two triangles per step between the edges
        fill = marker('detect_lane_fill', Marker.TRIANGLE_LIST)
        fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        fill.color.a = 0.25
        for i in range(len(steps) - 1):
            fill.points += [left[i], right[i], left[i + 1], left[i + 1], right[i], right[i + 1]]

        # red ball on the closest obstacle point, removed when the lane is clear
        ball = marker('closest_obstacle', Marker.SPHERE)
        if hit is not None:
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = hit[0], hit[1], 0.15
            ball.scale.x = ball.scale.y = ball.scale.z = 0.15
            ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.1, 0.1, 1.0
        else:
            ball.action = Marker.DELETE

        self.marker_pub.publish(MarkerArray(markers=[outline, fill, ball]))


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

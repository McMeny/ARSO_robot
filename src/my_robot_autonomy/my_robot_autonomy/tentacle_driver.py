"""Drives the walker directly from the tentacle fan - no map, no planner, no Nav2.

20 times a second:
  1. Where does the user want to go?  The (filtered) head direction, relative to the walker,
     scaled by how much say the head has (shared by risk = clearance straight ahead, as before).
     With no head input, or no say, "straight on".
  2. Pick a tentacle: only arcs clear for at least stop_distance + margin count. Among those,
     the one whose direction (after `lookahead` metres) is closest to where the user wants to go,
     with a preference for staying on the arc already being followed (no zig-zag) and for more
     open arcs. So it drives straight until something is in the way, then curves round it, and
     a wall ahead turns it smoothly to the open side.
  3. Speed: normal pace x the hazard cap (/speed_limit from walker_brain), slower as the chosen
     arc's free length shrinks and in tight curves; turn rate = speed x curvature. Smoothed by
     acceleration limits.
  4. Boxed in (no arc clear): turn slowly on the spot towards the more open side if nothing is
     within the walker's turning circle; otherwise reverse slowly if the space behind is clear;
     otherwise stop and wait.

Publishes cmd_vel (into twist_mux, so teleop and Stop walking still win), /plan (the chosen arc,
map frame - the hazard tracker and floor trajectory use it), and /machine_authority.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformListener, TransformException

from my_robot_autonomy.tentacles import arc_point
from walker_interfaces.msg import HazardArray, Tentacles

PERIOD = 0.05


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TentacleDriver(Node):
    def __init__(self):
        super().__init__('tentacle_driver')
        p = self.declare_parameter
        self.enabled = p('start_enabled', True).value
        self.max_speed = p('max_speed', 0.26).value          # m/s, normal walking pace for the walker
        self.max_turn = p('max_turn_rate', 0.8).value        # rad/s
        self.accel = p('accel', 0.25).value                  # m/s^2
        self.turn_accel = p('turn_accel', 1.0).value         # rad/s^2
        self.stop_distance = p('stop_distance', 0.25).value  # an arc blocked closer than this can't be used
        self.margin = p('margin', 0.3).value
        self.slow_distance = p('slow_distance', 1.5).value   # slow down as the free length drops below this
        self.lookahead = p('lookahead', 1.0).value           # m along an arc to judge its direction
        self.angle_weight = p('angle_weight', 0.5).value     # how much going the wanted way matters
        self.stick = p('continuity_weight', 0.6).value       # preference for the arc already followed
        self.open_weight = p('open_weight', 2.0).value       # preference for arcs with room ahead
        self.comfort = p('comfortable_free_length', 2.0).value  # m of free arc that counts as "open"
        self.side_weight = p('side_gap_weight', 1.0).value   # preference for arcs that keep away from things
        self.side_comfort = p('comfortable_side_gap', 0.45).value  # m beside the walker that counts as roomy
        # head-guided direction (same safeguards as before)
        self.head_filter_time = p('head_filter_time', 1.5).value
        self.head_deadband = math.radians(p('head_deadband_deg', 12.0).value)
        self.hazard_exclusion = math.radians(p('hazard_look_exclusion_deg', 10.0).value)
        self.open_clearance = p('open_clearance', 2.5).value
        self.near_clearance = p('near_clearance', 0.8).value
        self.head_timeout = p('head_timeout', 1.0).value
        # boxed in
        self.turn_circle = p('turn_circle_radius', 0.58).value  # the walker's corners when turning on the spot
        self.spin_speed = p('spin_speed', 0.3).value
        self.reverse_speed = p('reverse_speed', 0.05).value
        self.reverse_min_time = p('reverse_min_time', 2.5).value   # once backing out, keep going this long
        self.resume_extra = p('resume_extra', 0.25).value          # ...and only drive on with this much spare room

        self.tentacles = None
        self.hazards = []
        self.cap = 1.0
        self.head_filtered = None
        self.head_time = -math.inf
        self.k_prev = 0.0
        self.v = 0.0
        self.w = 0.0
        self.mode = 'drive'
        self.spin_dir = 0.0       # side chosen when boxed in - kept until the walker is free again
        self.reverse_since = None
        self.points = None        # lidar points in base_footprint, for the boxed-in checks
        self.laser_offset = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.authority_pub = self.create_publisher(Float32, 'machine_authority', 10)
        self.create_subscription(Tentacles, 'tentacles', lambda m: setattr(self, 'tentacles', m), 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(PoseStamped, 'head_pose', self.on_head, 10)
        self.create_subscription(Bool, 'walker_enabled', self.on_enabled, latched)
        self.create_subscription(SpeedLimit, 'speed_limit', self.on_speed_limit, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(PERIOD, self.update)
        self.get_logger().info('Tentacle driving ' + ('on' if self.enabled else '- waiting for Start walking'))

    # ---------- inputs ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_enabled(self, msg):
        self.enabled = msg.data

    def on_speed_limit(self, msg):
        self.cap = 1.0 if msg.speed_limit <= 0 else min(msg.speed_limit / 100.0, 1.0)

    def on_head(self, msg):
        yaw = yaw_of(msg.pose.orientation)
        hx, hy = msg.pose.position.x, msg.pose.position.y
        now = self.now()
        for h in self.hazards:
            if h.frequency > 0:
                bearing = math.atan2(h.position.y - hy, h.position.x - hx)
                if abs(wrap(bearing - yaw)) < self.hazard_exclusion:
                    self.head_time = now  # looking at a tag is the probe, not a choice of direction
                    return
        if self.head_filtered is None or now - self.head_time > self.head_timeout:
            self.head_filtered = yaw
        else:
            a = min((now - self.head_time) / self.head_filter_time, 1.0)
            self.head_filtered = wrap(self.head_filtered + a * wrap(yaw - self.head_filtered))
        self.head_time = now

    def on_scan(self, scan):
        if self.laser_offset is None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', scan.header.frame_id, rclpy.time.Time())
            except TransformException:
                return
            self.laser_offset = (t.transform.translation.x, t.transform.translation.y)
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        self.points = np.c_[self.laser_offset[0] + r[ok] * np.cos(a[ok]), self.laser_offset[1] + r[ok] * np.sin(a[ok])]

    # ---------- deciding ----------

    def head_weight(self):
        if self.head_filtered is None or self.now() - self.head_time > self.head_timeout or self.tentacles is None:
            return 0.0
        w = (self.tentacles.clearance - self.near_clearance) / (self.open_clearance - self.near_clearance)
        return max(0.0, min(1.0, w))

    def wanted_direction(self, yaw, weight):
        """Where the user wants to go, relative to the walker (rad, + = left)."""
        if weight <= 0.0:
            return 0.0
        look = wrap(self.head_filtered - yaw)
        if abs(look) <= self.head_deadband:
            return 0.0
        return math.copysign(abs(look) - self.head_deadband, look) * weight

    def pick(self, wanted):
        """The best clear tentacle, or None if every one is blocked."""
        t = self.tentacles
        best, best_cost = None, math.inf
        gaps = t.side_gaps if len(t.side_gaps) == len(t.curvatures) else [1.0] * len(t.curvatures)
        for k, free, gap in zip(t.curvatures, t.free_lengths, gaps):
            if free < self.stop_distance + self.margin:
                continue
            direction = max(-math.pi / 2, min(math.pi / 2, k * self.lookahead))
            cost = (self.angle_weight * abs(wrap(direction - wanted))
                    + self.stick * abs(k - self.k_prev) / max(abs(t.curvatures[0]), 1e-3)
                    + self.open_weight * max(0.0, self.comfort - free) / self.comfort
                    + self.side_weight * max(0.0, self.side_comfort - gap) / self.side_comfort)
            if cost < best_cost:
                best, best_cost = (k, free), cost
        return best

    def boxed_in_move(self):
        """No arc is clear: turn on the spot if the turning circle is empty, else back out slowly."""
        pts = self.points if self.points is not None else np.empty((0, 2))
        centre = np.array([0.21, 0.0])   # middle of the walker's body
        if not np.any(np.hypot(pts[:, 0] - centre[0], pts[:, 1] - centre[1]) < self.turn_circle):
            if self.spin_dir == 0.0:   # pick a side once, then keep turning that way
                t = self.tentacles
                left = sum(f for k, f in zip(t.curvatures, t.free_lengths) if k > 0)
                right = sum(f for k, f in zip(t.curvatures, t.free_lengths) if k < 0)
                self.spin_dir = 1.0 if left >= right else -1.0
            return 'turn', 0.0, self.spin_dir * self.spin_speed
        # straight back only sweeps the body's own width, so walls alongside don't count
        behind = pts[(pts[:, 0] < -0.12) & (pts[:, 0] > -0.7) & (np.abs(pts[:, 1]) < 0.28)]
        if len(behind) == 0:
            return 'reverse', -self.reverse_speed, 0.0
        return 'wait', 0.0, 0.0

    # ---------- main loop ----------

    def update(self):
        weight = self.head_weight()
        self.authority_pub.publish(Float32(data=1.0 - weight))
        if self.tentacles is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        x, y, yaw = tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)

        if not self.enabled:
            v_target, w_target, mode, k = 0.0, 0.0, 'stopped', 0.0
        else:
            choice = self.pick(self.wanted_direction(yaw, weight))
            # backing out: keep going for reverse_min_time, and until there's clearly room to drive on
            if self.mode == 'reverse' and self.reverse_since is not None:
                backing = self.now() - self.reverse_since < self.reverse_min_time
                roomy = choice is not None and choice[1] >= self.stop_distance + self.margin + self.resume_extra
                if backing or not roomy:
                    choice = None
            if choice is not None:
                k, free = choice
                room = max(0.0, min(1.0, (free - self.stop_distance) / self.slow_distance))
                bend = 1.0 - 0.5 * abs(k) / max(abs(self.tentacles.curvatures[0]), 1e-3)
                v_target = self.max_speed * self.cap * max(room, 0.25) * bend
                w_target = max(-self.max_turn, min(self.max_turn, v_target * k))
                mode = 'drive'
                self.spin_dir = 0.0
            else:
                mode, v_target, w_target = self.boxed_in_move()
                k = 0.0
                if mode == 'reverse' and self.mode != 'reverse':
                    self.reverse_since = self.now()
                elif mode != 'reverse':
                    self.reverse_since = None
            if mode != self.mode and mode != 'drive':
                self.get_logger().warn({'turn': 'Boxed in - turning slowly on the spot',
                                        'reverse': 'Boxed in - backing out slowly',
                                        'wait': 'Boxed in on all sides - waiting'}[mode])
        self.mode = mode
        self.k_prev = k

        # smooth the command
        dv = max(-self.accel * PERIOD, min(self.accel * PERIOD, v_target - self.v))
        dw = max(-self.turn_accel * PERIOD, min(self.turn_accel * PERIOD, w_target - self.w))
        self.v += dv
        self.w += dw
        if mode in ('stopped', 'wait'):
            self.v, self.w = 0.0, 0.0
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = self.v, self.w
        self.cmd_pub.publish(cmd)
        self.publish_plan(x, y, yaw, k if mode == 'drive' else 0.0)

    def publish_plan(self, x, y, yaw, k):
        """The chosen arc (3 m), then straight on (3 m more), in the map frame."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        length = self.tentacles.length if self.tentacles is not None else 3.0
        if abs(k) > 1e-3:
            length = min(length, 0.9 * math.pi / abs(k))  # don't curl round more than ~160 degrees
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [arc_point(k, d) for d in np.arange(0.0, length + 1e-9, 0.1)]
        ex, ey = pts[-1]
        end_heading = k * length
        pts += [(ex + d * math.cos(end_heading), ey + d * math.sin(end_heading)) for d in np.arange(0.1, 3.0 + 1e-9, 0.1)]
        for px, py in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x + c * px - s * py
            ps.pose.position.y = y + s * px + c * py
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.plan_pub.publish(path)


def main():
    rclpy.init()
    node = TentacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""The hazard loop's bookkeeping: which obstacles exist, which matter, which are tagged.

1. Detect - an object counts as detected once the lidar has hit it (min_hits points).
   The store's clutter (carts, pallets, boxes, signs...) are hazards; walls, shelves,
   checkouts and tables are structure. In this simulation the object list and their
   names come from the world file, standing in for an object classifier on a real walker.
2. Decide if it matters - relevant = ahead along the walker's path (Nav2's /plan),
   within probe_start_distance, and within near_path_margin of the path.
3. Tag - up to max_simultaneous relevant hazards get their own flicker frequency
   (harmonic clashes avoided); the rest wait in a queue.
4. Measure / act - the EEG decoder reports per hazard on /probe_results:
     present                          -> REGISTERED (tag fades, handled gently)
     not registered by decision_distance -> MISSED (stronger cue; walker_brain slows
                                        down and widens the berth). Keeps probing - if
                                        the response appears later it becomes REGISTERED.
   A hazard that's queued, or whose tag is switched off, and reaches the decision
   distance is MISSED too - no reliable answer means cautious.

/hazards (walker_interfaces/HazardArray) for walker_brain, the headset and the logger;
/hazard_markers for RViz.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_autonomy.world_model import load_world
from walker_interfaces.msg import Hazard, HazardArray, ProbeResult

STATE_NAMES = {Hazard.WAITING: 'waiting', Hazard.PROBING: 'probing', Hazard.REGISTERED: 'registered',
               Hazard.MISSED: 'missed', Hazard.PASSED: 'passed'}
STATE_COLOURS = {Hazard.WAITING: (0.6, 0.6, 0.6), Hazard.PROBING: (0.2, 0.6, 1.0),
                 Hazard.REGISTERED: (0.2, 0.9, 0.3), Hazard.MISSED: (1.0, 0.4, 0.1),
                 Hazard.PASSED: (0.4, 0.4, 0.4)}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Tracked:
    def __init__(self, obj, hid):
        self.obj = obj
        self.id = hid
        self.detected = False
        self.hits = 0
        self.state = Hazard.WAITING
        self.ever_registered = False
        self.frequency = 0.0
        self.contrast = 0.0
        self.confidence = 0.0
        self.path_distance = math.inf
        self.lateral = math.inf
        self.relevant = False
        self.off_path_since = None   # when an active probe's hazard first looked off the path
        self.probe_started = None    # when the current tag started flickering


class HazardTracker(Node):
    def __init__(self):
        super().__init__('hazard_tracker')
        p = self.declare_parameter
        world_file = p('world_file', '').value
        self.hazard_prefixes = list(p('hazard_name_prefixes', ['aisle', 'back_aisle', 'main_aisle']).value)
        self.probe_start = p('probe_start_distance', 4.0).value
        self.decision = p('decision_distance', 2.5).value
        self.margin = p('near_path_margin', 0.6).value
        self.max_simultaneous = p('max_simultaneous', 2).value
        self.frequencies = list(p('frequencies', [7.5, 12.86, 6.92, 6.43]).value)
        self.subtle = p('subtle_contrast', 0.25).value
        self.strong = p('strong_contrast', 0.5).value
        self.min_hits = p('min_hits', 3).value
        self.front = p('front_offset', 0.46).value
        self.off_path_time = p('off_path_time', 2.0).value  # s off the path before an active probe is dropped
        # a hazard that only shows up late still gets min_probe_time of flicker before it can be escalated,
        # unless it gets closer than last_decision_distance
        self.min_probe_time = p('min_probe_time', 3.0).value
        self.last_decision = p('last_decision_distance', 1.2).value

        objects = [o for o in load_world(world_file) if any(o.name.startswith(pre) for pre in self.hazard_prefixes)]
        self.tracked = [Tracked(o, i + 1) for i, o in enumerate(objects)]
        self.get_logger().info(f'{len(self.tracked)} possible hazards in the world: '
                               + ', '.join(t.obj.name for t in self.tracked))

        self.path = None            # Nav2's latest plan, as an (N, 2) array
        self.path_time = -math.inf
        self.flicker_enabled = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(HazardArray, 'hazards', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'hazard_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self.on_plan, 10)
        self.create_subscription(ProbeResult, 'probe_results', self.on_result, 50)
        self.create_subscription(Bool, 'flicker_enabled', self.on_flicker, latched)
        self.create_timer(0.2, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- inputs ----------

    def on_plan(self, msg):
        if msg.poses:
            self.path = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
            self.path_time = self.now()

    def on_flicker(self, msg):
        self.flicker_enabled = msg.data
        if not msg.data:
            self.get_logger().warn('Flicker switched OFF by the operator - no hazards can be probed')

    def on_scan(self, scan):
        """Mark objects the lidar has actually hit as detected."""
        undetected = [t for t in self.tracked if not t.detected]
        if not undetected:
            return
        try:
            tf = self.tf_buffer.lookup_transform('map', scan.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        ok = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        yaw = yaw_of(tf.transform.rotation)
        lx, ly = ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])
        mx = tf.transform.translation.x + math.cos(yaw) * lx - math.sin(yaw) * ly
        my = tf.transform.translation.y + math.sin(yaw) * lx + math.cos(yaw) * ly
        for t in undetected:
            near = np.hypot(mx - t.obj.x, my - t.obj.y) < max(t.obj.size_x, t.obj.size_y) / 2 + 0.3
            hits = sum(1 for px, py in zip(mx[near], my[near]) if t.obj.distance_to(px, py) < 0.1)
            t.hits = max(t.hits, hits)
            if t.hits >= self.min_hits:
                t.detected = True
                self.get_logger().info(f'Detected hazard {t.id} ({t.obj.name})')

    def on_result(self, msg):
        t = next((t for t in self.tracked if t.id == msg.hazard_id), None)
        if t is None or t.frequency == 0.0 or abs(t.frequency - msg.frequency) > 0.01:
            return  # result for a tag that has since changed
        t.confidence = msg.score
        if msg.present and t.state in (Hazard.PROBING, Hazard.MISSED):
            self.get_logger().info(f'Hazard {t.id} ({t.obj.name}) REGISTERED by the EEG (score {msg.score:.1f})')
            t.state = Hazard.REGISTERED
            t.ever_registered = True
            self.release(t)

    # ---------- geometry ----------

    def walker_path(self, x, y, yaw):
        """The path ahead as points from the walker: Nav2's plan if fresh, extended straight
        to probe_start + 2 m; otherwise straight ahead along the walker's heading."""
        reach = self.probe_start + self.front + 2.0
        if self.path is not None and self.now() - self.path_time < 3.0 and len(self.path) >= 2:
            start = int(np.argmin(np.hypot(self.path[:, 0] - x, self.path[:, 1] - y)))
            pts = [np.array([x, y])] + list(self.path[start:])
            if len(pts) >= 2:
                d = pts[-1] - pts[-2]
                n = np.linalg.norm(d)
                if n > 1e-6:
                    pts.append(pts[-1] + d / n * reach)
        else:
            pts = [np.array([x, y]), np.array([x + reach * math.cos(yaw), y + reach * math.sin(yaw)])]
        # resample every 0.1 m
        out, s_out = [pts[0]], [0.0]
        s = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            seg = np.linalg.norm(b - a)
            n = max(int(seg / 0.1), 1)
            for j in range(1, n + 1):
                out.append(a + (b - a) * j / n)
                s += seg / n
                s_out.append(s)
            if s > reach:
                break
        return np.array(out), np.array(s_out)

    def measure(self, t, pts, s):
        """Distance ahead (from the walker's front) and sideways gap between path and object."""
        gaps = np.array([t.obj.distance_to(px, py) for px, py in pts])
        i = int(np.argmin(gaps))
        t.lateral = float(gaps[i])
        t.path_distance = float(s[i]) - self.front

    # ---------- frequencies ----------

    def pick_frequency(self):
        used = [t.frequency for t in self.tracked if t.frequency > 0]
        for f in self.frequencies:
            clash = any(abs(f - u) < 0.3 or abs(2 * f - u) < 0.6 or abs(2 * u - f) < 0.6 for u in used)
            if not clash:
                return f
        return 0.0

    def release(self, t):
        t.frequency = 0.0
        t.contrast = 0.0

    # ---------- the loop ----------

    def update(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        x, y, yaw = tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation)
        pts, s = self.walker_path(x, y, yaw)

        for t in self.tracked:
            if not t.detected:
                continue
            self.measure(t, pts, s)
            active = t.state in (Hazard.PROBING, Hazard.MISSED)
            # once a probe has started it stays relevant through small changes in Nav2's plan
            # (wider margin, and only dropped after being off the path for off_path_time)
            margin = self.margin + (0.8 if active else 0.0)
            ahead = 0.0 < t.path_distance <= self.probe_start + (2.0 if active else 0.0)
            on_path = ahead and t.lateral <= margin
            if on_path or not active:
                t.off_path_since = None
            elif t.off_path_since is None:
                t.off_path_since = self.now()
            off_path = active and t.off_path_since is not None and self.now() - t.off_path_since > self.off_path_time
            t.relevant = (on_path or (active and not off_path)) and not t.ever_registered

            # behind the walker, or the path has clearly moved away -> passed (a missed one can be
            # probed again on a later pass)
            if t.path_distance <= -0.3 or off_path:
                if t.state in (Hazard.PROBING, Hazard.MISSED, Hazard.WAITING):
                    t.state = Hazard.REGISTERED if t.ever_registered else Hazard.PASSED
                t.off_path_since = None
                self.release(t)
                continue
            if t.state == Hazard.PASSED and t.relevant:
                t.state = Hazard.WAITING

        # queue: closest relevant hazards first
        candidates = sorted((t for t in self.tracked if t.detected and t.relevant
                             and t.state in (Hazard.WAITING, Hazard.PROBING, Hazard.MISSED)),
                            key=lambda t: t.path_distance)
        for t in candidates:
            if not self.flicker_enabled:
                self.release(t)
            elif t.frequency == 0.0 and t.state in (Hazard.WAITING, Hazard.MISSED):
                busy = sum(1 for o in self.tracked if o.frequency > 0)
                if busy < self.max_simultaneous:
                    f = self.pick_frequency()
                    if f > 0:
                        t.frequency = f
                        if t.state == Hazard.WAITING:
                            t.contrast, t.state = self.subtle, Hazard.PROBING
                            t.probe_started = self.now()
                        else:
                            t.contrast = self.strong  # missed while queued - tag it with the strong cue
                        self.get_logger().info(f'Probing hazard {t.id} ({t.obj.name}) at {f:.2f} Hz, '
                                               f'{t.path_distance:.1f} m ahead')
            # decision distance reached without a "present" -> escalate. A probe that started late gets
            # min_probe_time first (the decoder needs ~2-3 s), but never past last_decision_distance.
            probed_long_enough = (t.state != Hazard.PROBING or t.probe_started is None
                                  or self.now() - t.probe_started >= self.min_probe_time)
            too_close = t.path_distance <= self.last_decision
            if (t.state in (Hazard.WAITING, Hazard.PROBING) and t.path_distance <= self.decision
                    and (probed_long_enough or too_close)):
                t.state = Hazard.MISSED
                t.contrast = self.strong if t.frequency > 0 else 0.0
                self.get_logger().warn(f'Hazard {t.id} ({t.obj.name}) MISSED - not registered by '
                                       f'{self.decision} m, escalating')
            if t.state == Hazard.MISSED and t.frequency > 0:
                t.contrast = self.strong

        self.publish()

    def publish(self):
        msg = HazardArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        markers = []
        for t in self.tracked:
            if not t.detected:
                continue
            h = Hazard()
            h.id, h.name, h.kind = t.id, t.obj.name, t.obj.kind
            h.position.x, h.position.y, h.position.z = t.obj.x, t.obj.y, t.obj.z
            h.size_x, h.size_y, h.height, h.yaw = t.obj.size_x, t.obj.size_y, t.obj.height, t.obj.yaw
            h.path_distance = t.path_distance if math.isfinite(t.path_distance) else 99.0
            h.lateral = t.lateral if math.isfinite(t.lateral) else 99.0
            h.relevant, h.state = t.relevant, t.state
            h.frequency, h.contrast, h.confidence = t.frequency, t.contrast, t.confidence
            msg.hazards.append(h)
            markers += self.markers_for(t, msg.header)
        self.pub.publish(msg)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def markers_for(self, t, header):
        r, g, b = STATE_COLOURS[t.state]
        box = Marker()
        box.header = header
        box.ns, box.id, box.action = 'hazard', t.id, Marker.ADD
        box.type = Marker.CUBE if t.obj.kind == 'box' else Marker.CYLINDER
        box.pose.position.x, box.pose.position.y, box.pose.position.z = t.obj.x, t.obj.y, t.obj.z
        box.pose.orientation.z, box.pose.orientation.w = math.sin(t.obj.yaw / 2), math.cos(t.obj.yaw / 2)
        box.scale.x, box.scale.y, box.scale.z = t.obj.size_x + 0.08, t.obj.size_y + 0.08, t.obj.height + 0.08
        box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.45 if t.relevant else 0.2
        label = Marker()
        label.header = header
        label.ns, label.id, label.action = 'hazard_label', t.id, Marker.ADD
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y = t.obj.x, t.obj.y
        label.pose.position.z = t.obj.height + 0.35
        label.pose.orientation.w = 1.0
        label.scale.z = 0.2
        label.color.r, label.color.g, label.color.b, label.color.a = r, g, b, 1.0
        freq = f' {t.frequency:.2f} Hz' if t.frequency > 0 else ''
        label.text = f'#{t.id} {STATE_NAMES[t.state]}{freq}'
        return [box, label]


def main():
    rclpy.init()
    node = HazardTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

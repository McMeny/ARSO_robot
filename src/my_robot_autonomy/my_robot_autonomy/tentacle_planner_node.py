"""The tentacle planner - drives the walker directly from the lidar, 20 times a second.
No map, no Nav2 planner. The maths is in tentacle_core.py.

Each cycle:
  1. Grip state: STOP -> zero velocity. WALK -> keep going the way WALK started (straight unless
     obstacles force a curve, never turning more than walk_max_turn on its own). STEER -> head
     direction, its say (authority) shrinking as the clearance around the walker shrinks.
  2. Fan of tentacles (constant-curvature arcs), each checked by sweeping the walker+user
     footprint along it against the lidar. Lowest cost valid tentacle wins:
        clearance + head + hazard berths + hysteresis + curvature (+ return to the WALK heading)
  3. Speed: v_max x clearance ahead x caution (an unregistered or no-EEG hazard close ahead).
     Rate-limited here; the velocity smoother and the collision monitor come after this node.
  4. Turning in place: in STEER, nearly stopped, and the head more than pivot_angle away (or
     blocked and more than blocked_pivot_angle away) -> pivot, if the rotating footprint is clear.
  5. Blocked: no valid tentacle -> zero, and after a moment the notice "Path blocked. Squeeze to
     steer out".

The EEG never commands motion: it only reaches here through the hazards' registered / eeg_valid
flags, which change the berth and the speed.

In:  /scan, /odometry/filtered, /walker/grip_state, /head_pose (map frame), /hazards
Out: /cmd_vel_tentacle, /walker/trajectory (+ _left, _right), /walker/tentacles_debug,
     /walker/steering_status, /plan (chosen tentacle then straight on in the wanted direction,
     for the hazard tracker), /walker_status (JSON, for the operator page and the logger)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_autonomy.tentacle_core import (
    Footprint, Params, TentaclePlanner, HazardLite, HintLogic, STOP, WALK, STEER,
    wrap, front_lines, outline, rotation_poses, self_filter)
from walker_interfaces.msg import GripState, Hazard, HazardArray, SteeringStatus

PERIOD = 0.05
GRIP_NAMES = {STOP: 'STOP', WALK: 'WALK', STEER: 'STEER'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_2d(t):
    return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)


class TentaclePlannerNode(Node):
    def __init__(self):
        super().__init__('tentacle_planner')
        p = self.declare_parameter
        d = Params()
        params = Params(**{name: p(name, getattr(d, name)).value for name in d.__dataclass_fields__ if name != 'walk_max_turn'})
        params.walk_max_turn = math.radians(p('walk_max_turn_deg', math.degrees(d.walk_max_turn)).value)
        fp = Footprint(length=p('footprint_length', 1.25).value, width=p('footprint_width', 0.6).value,
                       offset_x=p('footprint_offset_x', -0.165).value, margin=p('safety_margin', 0.08).value)
        self.planner = TentaclePlanner(params, fp)
        self.fp = fp
        self.accel = p('accel', 0.3).value
        self.decel = p('decel', 0.3).value
        self.stop_decel = p('stop_decel', 0.5).value       # grip released, or blocked
        self.turn_accel = p('turn_accel', 0.8).value
        self.edge_half_width = p('edge_half_width', 0.25).value   # floor lines: the walker's own width
        self.display_extra = p('berth_display_extra', 0.12).value  # edges widen by this near an unregistered hazard
        self.self_filter = p('self_filter', True).value
        # head
        self.head_filter_time = p('head_filter_time', 0.6).value
        self.head_timeout = p('head_timeout', 1.0).value
        self.hazard_exclusion = math.radians(p('hazard_look_exclusion_deg', 10.0).value)
        # turning in place
        self.pivot_speed = p('pivot_speed_threshold', 0.05).value
        self.pivot_angle = math.radians(p('pivot_angle_deg', 60.0).value)
        self.blocked_pivot_angle = math.radians(p('blocked_pivot_angle_deg', 20.0).value)
        self.pivot_done = math.radians(p('pivot_done_angle_deg', 10.0).value)
        self.omega_pivot = p('omega_pivot', 0.4).value
        self.pivot_check = math.radians(p('pivot_check_deg', 30.0).value)
        self.hints = HintLogic(
            hint_angle=math.radians(p('hint_angle_deg', 30.0).value), hint_dwell_s=p('hint_dwell_s', 3.0).value,
            hint_cooldown_s=p('hint_cooldown_s', 30.0).value, hint_max_successes=p('hint_max_successes', 3).value,
            blocked_notice_delay_s=p('blocked_notice_delay_s', 1.0).value)

        self.points = np.empty((0, 2))
        self.scan_time = -math.inf
        self.laser_offset = None
        self.v_odom = 0.0
        self.grip = STOP
        self.grip_time = -math.inf
        self.hazards = []
        self.head_yaw = None        # filtered, map frame
        self.head_time = -math.inf
        self.ref_heading = None     # the heading WALK keeps to
        self.k_prev = 0.0
        self.v = self.w = 0.0
        self.pivoting = False
        self.last_mode = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tentacle', 10)
        self.traj_pub = self.create_publisher(Path, 'walker/trajectory', 10)
        self.left_pub = self.create_publisher(Path, 'walker/trajectory_left', 10)
        self.right_pub = self.create_publisher(Path, 'walker/trajectory_right', 10)
        self.debug_pub = self.create_publisher(MarkerArray, 'walker/tentacles_debug', 10)
        self.status_pub = self.create_publisher(SteeringStatus, 'walker/steering_status', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.walker_status_pub = self.create_publisher(String, 'walker_status', latched)
        self.authority_pub = self.create_publisher(Float32, 'machine_authority', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/filtered', lambda m: setattr(self, 'v_odom', m.twist.twist.linear.x), 10)
        self.create_subscription(GripState, 'walker/grip_state', self.on_grip, 10)
        self.create_subscription(PoseStamped, 'head_pose', self.on_head, 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_timer(PERIOD, self.update)
        self.get_logger().info(f'Tentacle planner: {params.num_tentacles} tentacles, v_max {params.v_max} m/s')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------- inputs ----------

    def on_scan(self, scan):
        if self.laser_offset is None:
            try:
                t = self.tf_buffer.lookup_transform('base_footprint', scan.header.frame_id, rclpy.time.Time())
            except TransformException:
                return
            self.laser_offset = transform_2d(t)
        lx, ly, lyaw = self.laser_offset
        r = np.asarray(scan.ranges)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment + lyaw
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        pts = np.c_[lx + r[ok] * np.cos(a[ok]), ly + r[ok] * np.sin(a[ok])]
        self.points = self_filter(pts, self.fp) if self.self_filter else pts
        self.scan_time = self.now()

    def on_grip(self, msg):
        self.grip = msg.state
        self.grip_time = self.now()

    def on_head(self, msg):
        yaw = yaw_of(msg.pose.orientation)
        hx, hy = msg.pose.position.x, msg.pose.position.y
        now = self.now()
        for h in self.hazards:
            if h.frequency > 0 and abs(wrap(math.atan2(h.position.y - hy, h.position.x - hx) - yaw)) < self.hazard_exclusion:
                self.head_time = now   # looking at a tag is the probe, not a choice of direction
                return
        if self.head_yaw is None or now - self.head_time > self.head_timeout:
            self.head_yaw = yaw
        else:
            a = min((now - self.head_time) / self.head_filter_time, 1.0)
            self.head_yaw = wrap(self.head_yaw + a * wrap(yaw - self.head_yaw))
        self.head_time = now

    # ---------- helpers ----------

    def hazards_in_base(self, base_from_map):
        bx, by, byaw = base_from_map
        c, s = math.cos(byaw), math.sin(byaw)
        out = []
        for h in self.hazards:
            if h.state == Hazard.PASSED:
                continue
            x, y = h.position.x, h.position.y
            radius = h.radius if h.radius > 0 else math.hypot(h.size_x, h.size_y) / 2
            out.append(HazardLite(bx + c * x - s * y, by + s * x + c * y, radius,
                                  not h.registered or not h.eeg_valid))
        return out

    def path_msg(self, pts, frame='odom', pose=(0.0, 0.0, 0.0)):
        """A Path from (N, 2) points in base_footprint, placed in `frame` at `pose`."""
        x0, y0, th = pose
        c, s = math.cos(th), math.sin(th)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame
        for px, py in pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x0 + c * px - s * py
            ps.pose.position.y = y0 + s * px + c * py
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path

    # ---------- main loop ----------

    def update(self):
        now = self.now()
        try:
            odom_pose = transform_2d(self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time()))
            map_from_odom = transform_2d(self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time()))
            base_from_map = transform_2d(self.tf_buffer.lookup_transform('base_footprint', 'map', rclpy.time.Time()))
        except TransformException:
            return
        heading = odom_pose[2]
        grip = self.grip if now - self.grip_time < 0.5 else STOP      # no grip messages: STOP
        if now - self.scan_time > 0.5:
            grip = STOP                                                 # no lidar: don't move
        points = self.points

        # what the user wants
        head_ok = self.head_yaw is not None and now - self.head_time < self.head_timeout
        head_odom = wrap(self.head_yaw - map_from_odom[2]) if head_ok else None
        if grip == STEER or self.ref_heading is None or grip == STOP:
            self.ref_heading = heading       # WALK keeps the heading it starts with (or STEER left it at)
        if grip == STEER:
            desired = head_odom if head_odom is not None else heading
        else:
            desired = self.ref_heading
        hazards = self.hazards_in_base(base_from_map)

        choice = self.planner.choose(points, self.v_odom, heading, desired, grip, self.k_prev, hazards,
                                     self.ref_heading if grip == WALK else None)
        error = wrap(desired - heading)

        # mode
        mode = 'drive'
        v_target, w_target = self.planner.speed(points, choice)
        decel = self.decel
        if grip == STOP:
            mode, v_target, w_target, decel = 'stopped', 0.0, 0.0, self.stop_decel
            self.pivoting = False
        elif grip == STEER and head_odom is not None:
            start = self.pivot_angle if not choice.blocked else self.blocked_pivot_angle
            if self.pivoting and abs(error) < self.pivot_done:
                self.pivoting = False
            elif not self.pivoting and abs(error) > start:
                if abs(self.v) < self.pivot_speed:
                    self.pivoting = True
                else:
                    v_target, w_target = 0.0, 0.0   # slow down first, then turn on the spot
            if self.pivoting:
                step = math.copysign(min(self.pivot_check, abs(error)), error)
                if self.planner.pivot_clear(points, step):
                    mode, v_target, w_target = 'pivot', 0.0, math.copysign(self.omega_pivot, error)
                else:
                    self.pivoting = False
        else:
            self.pivoting = False
        if mode == 'drive' and choice.blocked:
            mode, decel = 'blocked', self.stop_decel
        if mode in ('drive', 'blocked'):
            self.k_prev = choice.k if not choice.blocked else self.k_prev

        # rate limits
        limit = (self.accel if v_target > self.v else decel) * PERIOD
        self.v += max(-limit, min(limit, v_target - self.v))
        self.w += max(-self.turn_accel * PERIOD, min(self.turn_accel * PERIOD, w_target - self.w))
        if mode == 'pivot':
            self.v = 0.0
        if mode in ('stopped', 'blocked') and abs(self.v) < 1e-3:
            self.v = 0.0
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(self.v), float(self.w)
        self.cmd_pub.publish(cmd)

        blocked = mode == 'blocked' or (mode == 'stopped' and choice.blocked and grip != STOP)
        look = wrap(self.head_yaw - map_from_odom[2] - heading) if head_ok else None
        hint, notice = self.hints.update(now, grip, look, choice.caution, choice.blocked and grip != STOP
                                         and mode != 'pivot', heading)
        if mode != self.last_mode:
            self.get_logger().info(f'{GRIP_NAMES[grip]}: {mode}' + (f' - {notice}' if notice else ''))
            self.last_mode = mode

        self.publish_outputs(mode, choice, grip, desired, error, odom_pose, map_from_odom, hint, notice, blocked)

    # ---------- outputs ----------

    def publish_outputs(self, mode, choice, grip, desired, error, odom_pose, map_from_odom, hint, notice, blocked):
        fp = self.fp
        half = self.edge_half_width + (self.display_extra if choice.near_unregistered else 0.0)
        show = max(abs(self.v) * self.planner.p.horizon_s, 0.8)  # next 2 s, at least 0.8 m
        if mode == 'pivot':
            # the footprint where the turn will end, and the arcs its front corners sweep
            poses = rotation_poses(error)
            centre = outline(poses, fp)
            _, left, right = front_lines(poses, fp, self.edge_half_width, 99)
        elif blocked:
            # a short footprint outline just in front of the walker (shown amber)
            centre = outline(np.zeros((1, 3)), fp, x_from=fp.front, x_to=fp.front + 0.3, half_width=self.edge_half_width)
            left = right = np.empty((0, 2))
        else:
            k, kind = (choice.k, choice.kind) if not choice.blocked else (0.0, 'arc')
            poses = self.planner.poses(k, self.planner.p.L_max, kind)
            centre, left, right = front_lines(poses, fp, half, show)
        self.traj_pub.publish(self.path_msg(centre, pose=odom_pose))
        self.left_pub.publish(self.path_msg(left, pose=odom_pose))
        self.right_pub.publish(self.path_msg(right, pose=odom_pose))

        status = SteeringStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'odom'
        status.grip_state = grip
        status.authority = float(choice.authority)
        status.desired_yaw = float(desired)
        status.turning_in_place = mode == 'pivot'
        status.blocked = blocked
        status.caution = choice.caution
        status.hint, status.notice = hint, notice
        self.status_pub.publish(status)
        self.authority_pub.publish(Float32(data=1.0 - choice.authority))

        # /plan for the hazard tracker: the chosen tentacle, then straight on in the wanted direction
        x, y, th = odom_pose
        mx, my, myaw = map_from_odom
        base_in_map = (mx + math.cos(myaw) * x - math.sin(myaw) * y, my + math.sin(myaw) * x + math.cos(myaw) * y, myaw + th)
        k, kind = (choice.k, choice.kind) if mode == 'drive' else (0.0, 'arc')
        arc = self.planner.poses(k, choice.length, kind)
        ex, ey = arc[-1, :2]
        rel = wrap(desired - th)
        tail = [(ex + s * math.cos(rel), ey + s * math.sin(rel)) for s in np.arange(0.2, 5.01, 0.2)]
        self.plan_pub.publish(self.path_msg(list(arc[:, :2]) + tail, 'map', base_in_map))

        caution = choice.caution
        cap = 100.0 * (self.planner.p.caution_scale if caution else 1.0)
        self.walker_status_pub.publish(String(data=json.dumps({
            'state': {'stopped': 'STOPPED', 'pivot': 'TURNING', 'blocked': 'BLOCKED'}.get(
                mode, 'CAREFUL' if caution else 'NORMAL'),
            'enabled': grip != STOP, 'grip': GRIP_NAMES[grip],
            'speed_cap_percent': cap, 'extra_clearance': self.display_extra if choice.near_unregistered else 0.0,
            'missed_hazards': [h.id for h in self.hazards if h.state == Hazard.MISSED],
            'machine_authority': round(1.0 - choice.authority, 2), 'hint': hint, 'notice': notice})))

        # debug fan: every tentacle coloured by cost (green = cheapest, red = dearest, dark = invalid)
        markers = []
        valid_costs = [c for c in choice.costs if math.isfinite(c)]
        lo, hi = (min(valid_costs), max(valid_costs)) if valid_costs else (0.0, 1.0)
        for i, ((kk, kind), cost) in enumerate(zip(self.planner.tentacles, choice.costs)):
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'tentacles', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            chosen = i == choice.index
            m.scale.x = 0.04 if chosen else 0.015
            if not math.isfinite(cost):
                m.color.r, m.color.g, m.color.b, m.color.a = 0.4, 0.0, 0.0, 0.5
            else:
                f = (cost - lo) / (hi - lo) if hi > lo else 0.0
                m.color.r, m.color.g, m.color.b, m.color.a = f, 1.0 - f, 1.0 if chosen else 0.1, 0.9
            m.points = [Point(x=float(px), y=float(py), z=0.03) for px, py, _ in self.planner.poses(kk, choice.length, kind)[::3]]
            markers.append(m)
        foot = Marker()
        foot.header.frame_id = 'base_footprint'
        foot.ns, foot.id, foot.type, foot.action = 'footprint', 0, Marker.LINE_STRIP, Marker.ADD
        foot.pose.orientation.w = 1.0
        foot.scale.x = 0.02
        foot.color.r = foot.color.g = 1.0
        foot.color.a = 0.8
        foot.points = [Point(x=float(px), y=float(py), z=0.02) for px, py in outline(np.zeros((1, 3)), self.fp)]
        markers.append(foot)
        self.debug_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = TentaclePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

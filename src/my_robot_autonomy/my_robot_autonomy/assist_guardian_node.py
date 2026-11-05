"""ASSIST + guardian - the user moves the walker, the walker protects them. 20 times a second.
(The maths is in guardian_core.py; the tentacle machinery in tentacle_core.py.)

  1. Dead-man: the grip (walker/grip_state, from grip_state_node) must be held. Let go -> locked at zero.
  2. ASSIST: handle forces (walker/handle_force) -> the user's intended speed and turn, through a
     virtual cart (admittance, ported from main). No push -> no motion. The walker never moves on its own.
  3. Guardian: checks the user's own path with the swept walker+user footprint against the lidar. If
     something is in the way it brakes, and/or bends the path by holding one wheel back - never
     faster than the push on either wheel, never more than 35 deg away from where the push points,
     and it undoes its own correction once clear. How early and firmly it steps in comes from the EEG
     via the hazards (registered / eeg_valid): a missed hazard gets help sooner, a wider berth and a
     lower speed cap.

The EEG never commands motion: it only changes how the guardian holds the user's push back.

In:  /scan, /walker/grip_state, /walker/handle_force, /hazards
Out: /cmd_vel_guardian (-> velocity smoother -> twist_mux -> collision monitor),
     /walker/intended_path (where the push alone is taking the user),
     /walker/trajectory (+ _left, _right: where the walker will actually go), /walker/tentacles_debug,
     /walker/steering_status, /plan (the intended path, for the hazard tracker), /walker_status (JSON)
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_autonomy.guardian_core import Admittance, DeadMan, Guardian, GuardParams, deadband
from my_robot_autonomy.tentacle_core import (Footprint, HazardLite, Params, front_lines, outline, rotation_poses,
                                             self_filter, shape_poses)
from walker_interfaces.msg import GripState, HandleForce, Hazard, HazardArray, SteeringStatus

PERIOD = 0.05
GRIP_NAMES = {GripState.STOP: 'STOP', GripState.WALK: 'HELD', GripState.STEER: 'HELD'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_2d(t):
    return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)


class AssistGuardianNode(Node):
    def __init__(self):
        super().__init__('assist_guardian')
        p = self.declare_parameter
        g = GuardParams()
        gparams = GuardParams(**{name: p(name, getattr(g, name)).value for name in g.__dataclass_fields__})
        fp = Footprint(length=p('footprint_length', 1.25).value, width=p('footprint_width', 0.6).value,
                       offset_x=p('footprint_offset_x', -0.165).value, margin=p('safety_margin', 0.12).value)
        self.fp = fp
        self.guardian = Guardian(gparams, fp, Params(L_min=gparams.L_min, L_max=gparams.L_max,
                                                     required_free=p('required_free', 0.7).value))
        self.adm = Admittance(mass=p('mass', 25.0).value, damping=p('damping', 40.0).value,
                              yaw_inertia=p('yaw_inertia', 4.0).value, yaw_damping=p('yaw_damping', 8.0).value,
                              force_deadband=p('force_deadband', 4.0).value, handle_y=p('handle_y', 0.25).value,
                              v_max=gparams.v_max, w_max=p('w_max', 0.8).value)
        self.deadman = DeadMan(p('release_timeout_s', 0.1).value)   # grip_state_node adds its own 0.2 s grace
        self.force_timeout = p('force_timeout', 0.3).value
        self.notice_delay = p('blocked_notice_delay_s', 1.0).value
        self.edge_half_width = p('edge_half_width', 0.25).value
        self.display_extra = p('berth_display_extra', 0.12).value
        self.self_filter = p('self_filter', True).value

        self.points = np.empty((0, 2))
        self.scan_time = -math.inf
        self.laser_offset = None
        self.grip = GripState.STOP
        self.grip_time = -math.inf
        self.force = (0.0, 0.0)
        self.force_time = -math.inf
        self.hazards = []
        self.blocked_since = None
        self.last_mode = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_guardian', 10)
        self.intended_pub = self.create_publisher(Path, 'walker/intended_path', 10)
        self.traj_pub = self.create_publisher(Path, 'walker/trajectory', 10)
        self.left_pub = self.create_publisher(Path, 'walker/trajectory_left', 10)
        self.right_pub = self.create_publisher(Path, 'walker/trajectory_right', 10)
        self.debug_pub = self.create_publisher(MarkerArray, 'walker/tentacles_debug', 10)
        self.status_pub = self.create_publisher(SteeringStatus, 'walker/steering_status', 10)
        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        self.walker_status_pub = self.create_publisher(String, 'walker_status', latched)
        self.authority_pub = self.create_publisher(Float32, 'machine_authority', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(GripState, 'walker/grip_state', self.on_grip, 10)
        self.create_subscription(HandleForce, 'walker/handle_force', self.on_force, 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_timer(PERIOD, self.update)
        self.get_logger().info('ASSIST + guardian: the walker moves only when pushed')

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

    def on_force(self, msg):
        self.force = (float(msg.left), float(msg.right))
        self.force_time = self.now()

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
        gripped = now - self.grip_time < 0.5 and self.grip != GripState.STOP
        locked = self.deadman.update(gripped and now - self.scan_time < 0.5, now)   # no lidar: locked too
        fl, fr = self.force if now - self.force_time < self.force_timeout else (0.0, 0.0)
        if locked:
            self.adm.reset()
            fl = fr = 0.0
        v_u, w_u = self.adm.step(fl, fr, PERIOD)
        pushing = deadband(fl, self.adm.db) != 0 or deadband(fr, self.adm.db) != 0
        hazards = self.hazards_in_base(base_from_map)
        d = self.guardian.decide(self.points, v_u, w_u, hazards, held=not locked, dt=PERIOD, pushing=pushing)
        self.adm.follow(d.v)

        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(d.v), float(d.w)
        self.cmd_pub.publish(cmd)

        # notice: pushing, but nothing that way is clear, for a moment
        if d.blocked and pushing and not locked:
            self.blocked_since = self.blocked_since if self.blocked_since is not None else now
        else:
            self.blocked_since = None
        notice = 'Blocked ahead. Turn to get round it' if (
            self.blocked_since is not None and now - self.blocked_since >= self.notice_delay) else ''
        mode = ('locked' if locked else 'blocked' if notice else 'turning' if d.turning_in_place and d.w
                else 'idle' if abs(v_u) < 0.02 and abs(w_u) < 0.02 else 'braking' if d.braking
                else 'nudging' if d.nudging else 'free')
        if mode != self.last_mode:
            self.get_logger().info(f'{mode}' + (f' - {notice}' if notice else ''))
            self.last_mode = mode
        self.publish_outputs(d, mode, notice, locked, odom_pose, map_from_odom)

    # ---------- outputs ----------

    def publish_outputs(self, d, mode, notice, locked, odom_pose, map_from_odom):
        fp = self.fp
        step = 0.05
        speed = max(abs(d.v_user), abs(d.v))
        show = max(speed * 2.0, 0.8)                       # the next 2 s, at least 0.8 m
        # where the push alone goes, and where the walker will actually go
        intended = shape_poses(d.k_user, 'arc', 2.5, step)
        actual = shape_poses(d.k_out if d.nudging else d.k_user, 'arc', 2.5, step)
        if d.v_user < 0:
            intended = np.c_[-intended[:, 0], intended[:, 1], -intended[:, 2]]
            actual = intended
        half = self.edge_half_width + (self.display_extra if d.near_unregistered else 0.0)
        if mode == 'turning':
            poses = rotation_poses(math.copysign(math.radians(45), d.w))
            centre = outline(poses, fp)
            _, left, right = front_lines(poses, fp, self.edge_half_width, 99)
        elif mode in ('blocked', 'locked') or (d.braking and d.v < 0.02):
            centre = outline(np.zeros((1, 3)), fp, x_from=fp.front, x_to=fp.front + 0.3, half_width=self.edge_half_width)
            left = right = np.empty((0, 2))
        else:
            # the actual path stops where the guardian will have stopped it
            stop = d.free if d.braking else 99.0
            centre, left, right = front_lines(actual, fp, half, min(show, stop))
        user_line, _, _ = front_lines(intended, fp, 0.0, show)
        if mode in ('idle', 'locked'):
            user_line = np.empty((0, 2))
        self.intended_pub.publish(self.path_msg(user_line, pose=odom_pose))
        self.traj_pub.publish(self.path_msg(centre, pose=odom_pose))
        self.left_pub.publish(self.path_msg(left, pose=odom_pose))
        self.right_pub.publish(self.path_msg(right, pose=odom_pose))

        st = SteeringStatus()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'odom'
        st.grip_state = GripState.STOP if locked else GripState.WALK
        st.authority = float(1.0 - d.intervention)
        st.desired_yaw = float(odom_pose[2] + d.k_user * 1.0)
        st.turning_in_place = mode == 'turning'
        st.blocked = mode == 'blocked'
        st.caution = d.caution
        st.notice = notice
        st.user_speed = float(d.v_user)
        st.intervention = float(d.intervention)
        st.braking, st.nudging = d.braking, d.nudging
        self.status_pub.publish(st)
        self.authority_pub.publish(Float32(data=float(d.intervention)))

        # /plan for the hazard tracker: where the user is heading (the intended path, then straight on)
        x, y, th = odom_pose
        mx, my, myaw = map_from_odom
        base_in_map = (mx + math.cos(myaw) * x - math.sin(myaw) * y, my + math.sin(myaw) * x + math.cos(myaw) * y, myaw + th)
        ex, ey, eth = intended[-1]
        tail = [(ex + s * math.cos(eth), ey + s * math.sin(eth)) for s in np.arange(0.2, 5.01, 0.2)]
        self.plan_pub.publish(self.path_msg(list(intended[:, :2]) + tail, 'map', base_in_map))

        self.walker_status_pub.publish(String(data=json.dumps({
            'state': {'locked': 'STOPPED', 'idle': 'WAITING FOR A PUSH', 'blocked': 'BLOCKED', 'turning': 'TURNING',
                      'braking': 'BRAKING', 'nudging': 'STEERING ROUND'}.get(mode, 'CAREFUL' if d.caution else 'FREE'),
            'enabled': not locked, 'grip': 'STOP' if locked else 'HELD',
            'user_speed': round(d.v_user, 2), 'speed': round(d.v, 2),
            'speed_cap_percent': round(100 * self.guardian.p.caution_scale, 0) if d.caution else 100.0,
            'extra_clearance': self.display_extra if d.near_unregistered else 0.0,
            'missed_hazards': [h.id for h in self.hazards if h.state == Hazard.MISSED],
            'machine_authority': round(d.intervention, 2), 'intervention': round(d.intervention, 2),
            'hint': '', 'notice': notice})))

        # debug: the user's tentacle (white) and the walker's (green / orange when corrected)
        markers = []
        for i, (poses, colour, width) in enumerate((
                (intended, (1.0, 1.0, 1.0), 0.02),
                (actual, (1.0, 0.6, 0.1) if d.nudging or d.braking else (0.2, 1.0, 0.3), 0.04))):
            m = Marker()
            m.header.frame_id = 'base_footprint'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = 'guardian', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = (*colour, 0.9)
            m.points = [Point(x=float(px), y=float(py), z=0.03) for px, py, _ in poses[::3]]
            markers.append(m)
        foot = Marker()
        foot.header.frame_id = 'base_footprint'
        foot.ns, foot.id, foot.type, foot.action = 'footprint', 0, Marker.LINE_STRIP, Marker.ADD
        foot.pose.orientation.w = 1.0
        foot.scale.x = 0.02
        foot.color.r = foot.color.g = 1.0
        foot.color.a = 0.8
        foot.points = [Point(x=float(px), y=float(py), z=0.02) for px, py in outline(np.zeros((1, 3)), fp)]
        markers.append(foot)
        self.debug_pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = AssistGuardianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

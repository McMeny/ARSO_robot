"""mujoco_bridge: MuJoCo as a hardware backend for the real ROS 2 stack (D15, D20).

backend:=mujoco      the bridge is the "robot driver": /cmd_vel in (same limits + 200 ms watchdog as the
                     firmware) -> wheels; publishes /odom + odom->base_footprint TF (from simulated
                     encoders, so odometry drifts), /joint_states, walker handle forces + grip.
backend:=mujoco_hil  the MCU emulator (the firmware's own C core on a pty) drives the wheels; the real
                     ros2_control + diffdrive_arduino path talks to it through mcu_bridge_node.
Both:                /clock, /scan (laser_frame, 10 Hz, sigma 1 cm, robot/user/MR geoms excluded), /imu,
                     /sim/ground_truth (evaluation only), /sim/people; MR-overlay markers in the viewer.
"""
from __future__ import annotations

import math
import threading
import time

from eyesteer_mujoco import _root  # noqa: F401  (puts the repo on sys.path)

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose2D, PoseArray, Pose, TransformStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Int32
from tf2_ros import TransformBroadcaster

from eyesteer_core.control.diffdrive_kinematics import EncoderOdometry
from eyesteer_core.world import load_world
from standalone.sim.simworld import SimWorld

from eyesteer_interfaces.msg import MarkerArray, RobotStatus


def stamp(t: float) -> Time:
    s = int(t)
    return Time(sec=s, nanosec=int((t - s) * 1e9))


def yaw_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class MujocoBridge(Node):
    def __init__(self):
        super().__init__("mujoco_bridge")
        p = self.declare_parameter
        self.world_name = p("world", "field").value
        self.robot = p("robot", "field_robot").value
        self.backend = p("backend", "mujoco").value
        self.viewer_on = p("viewer", False).value
        self.rtf = p("realtime_factor", 1.0).value
        self.mcu_link = p("mcu_link", "/tmp/eyesteer_mcu").value
        self.cmd_timeout = p("cmd_vel_timeout", 0.2).value       # D9: same as the firmware watchdog
        self.lidar_rate = p("lidar_rate", 10.0).value
        start = list(p("start_pose", [0.0, 0.0, 0.0]).value)
        world = load_world(self.world_name)
        if start == [0.0, 0.0, 0.0]:
            start = list(world.start_pose)
        self.sim = SimWorld(self.world_name, self.robot, seed=p("seed", 1).value, start_pose=start)
        self.is_walker = self.robot == "walker"
        x, y, yaw = self.sim.pose()
        self.odom = EncoderOdometry(self.sim.wr, self.sim.ws)   # odom frame starts at the robot (0, 0, 0)
        self.odom.update(*self.sim.wheel_angles(), 0.0)
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.cmd_t = -1e9
        self.forces = (0.0, 0.0)
        self.grip = True
        self.estop_button = False
        self.markers = None
        self.status = None

        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", rel)   # reliable: compatible with every subscriber QoS
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        self.pub_gt = self.create_publisher(Odometry, "/sim/ground_truth", 10)
        self.pub_people = self.create_publisher(PoseArray, "/sim/people", 10)
        self.tfb = TransformBroadcaster(self)
        if self.backend == "mujoco":
            self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
            self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
            self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
            if self.is_walker:
                self.pub_fl = self.create_publisher(WrenchStamped, "/handle_force/left", 10)
                self.pub_fr = self.create_publisher(WrenchStamped, "/handle_force/right", 10)
                self.pub_grip = self.create_publisher(Bool, "/grip", 10)
        else:
            import sys
            sys.path.insert(0, str(_root.ROOT / "firmware/emulator"))
            from mcu_emulator import McuEmulator
            self.emu = McuEmulator(self.robot, self.mcu_link, backend=self.sim, time_fn=lambda: self.sim.t)
            path = self.emu.open()
            self.emu_thread = threading.Thread(target=self._emu_loop, daemon=True)
            self.emu_thread.start()
            self.get_logger().info(f"HIL: firmware core emulated on {path}")
        # sim inputs (scenario_player / viewer keys / tests)
        self.create_subscription(Float32MultiArray, "/sim/handle_force_cmd", self.on_force, 10)
        self.create_subscription(Bool, "/sim/grip_cmd", lambda m: setattr(self, "grip", m.data), 10)
        self.create_subscription(Bool, "/sim/estop_button", self.on_estop_button, 10)
        self.create_subscription(Point, "/sim/spawn_obstacle", lambda m: self.sim.spawn_obstacle(m.x, m.y, m.z or 0.25), 10)
        self.create_subscription(Pose2D, "/sim/teleport", lambda m: self._teleport(m), 10)
        self.create_subscription(Bool, "/sim/kill_mcu", self.on_kill_mcu, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(MarkerArray, "/markers", lambda m: setattr(self, "markers", m), latched)
        self.create_subscription(RobotStatus, "/robot/status", lambda m: setattr(self, "status", m), 10)
        self.pub_gaze = self.create_publisher(Int32, "/bci/gaze", 10)
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f"mujoco_bridge: world={self.world_name} robot={self.robot} backend={self.backend} "
                               f"start=({start[0]:.2f}, {start[1]:.2f}, {start[2]:.2f})")

    # ------------------------------------------------------------------ inputs
    def on_cmd(self, m: Twist):
        with self.lock:
            self.cmd, self.cmd_t = (m.linear.x, m.angular.z), self.sim.t

    def on_force(self, m: Float32MultiArray):
        if len(m.data) >= 2:
            self.forces = (float(m.data[0]), float(m.data[1]))

    def on_estop_button(self, m: Bool):
        self.estop_button = m.data
        if self.backend == "mujoco":
            self.pub_estop.publish(Bool(data=m.data))   # no MCU: the button maps straight to /estop
        else:
            self.emu.estop_pressed = m.data

    def on_kill_mcu(self, m: Bool):
        if self.backend != "mujoco":
            self.emu.alive = not m.data
            if m.data:
                self.get_logger().warn("MCU emulator 'crashed' (failure injection)")

    def _teleport(self, m: Pose2D):
        with self.lock:
            self.sim.teleport(m.x, m.y, m.theta)

    def _emu_loop(self):
        self.emu.serve_forever()

    # ------------------------------------------------------------------ physics loop
    def _loop(self):
        dt = 0.01
        t_lidar = 0.0
        t_pub = 0.0
        t_slow = 0.0
        wall0, sim0 = time.monotonic(), self.sim.t
        viewer = None
        if self.viewer_on:
            try:
                import mujoco.viewer
                viewer = mujoco.viewer.launch_passive(self.sim.m, self.sim.d, key_callback=self.on_key)
                viewer.opt.geomgroup[3] = viewer.opt.geomgroup[4] = 1
            except Exception as e:  # no display
                self.get_logger().warn(f"viewer unavailable: {e}")
        while self.running and rclpy.ok():
            with self.lock:
                t = self.sim.t
                if self.backend == "mujoco":
                    v, w = self.cmd if (t - self.cmd_t) <= self.cmd_timeout else (0.0, 0.0)   # watchdog
                    self.sim.set_twist(v, w, dt)
                elif not self.emu.alive:
                    self.sim.set_wheels(0.0, 0.0)   # crashed MCU: H-bridge outputs float -> wheels stop
                if self.backend != "mujoco":
                    self.emu.forces, self.emu.grip = self.forces, self.grip
                self.sim.step(dt)
                t = self.sim.t
            self.pub_clock.publish(Clock(clock=stamp(t)))
            if t - t_pub >= 0.02:
                t_pub = t
                self._publish_state(t)
            if t - t_lidar >= 1.0 / self.lidar_rate:
                t_lidar = t
                self._publish_scan(t)
            if t - t_slow >= 0.1:
                t_slow = t
                self._publish_slow(t)
                if viewer is not None:
                    self._style_markers()
            if viewer is not None:
                if not viewer.is_running():
                    viewer = None
                else:
                    viewer.sync()
            # real-time pacing
            target = wall0 + (t - sim0) / max(self.rtf, 1e-3)
            sl = target - time.monotonic()
            if sl > 0:
                time.sleep(sl)
            elif sl < -0.5:
                wall0, sim0 = time.monotonic(), t   # fell behind: resync instead of sprinting

    def _publish_state(self, t):
        st = stamp(t)
        x, y, yaw = self.sim.pose()
        v, w = self.sim.twist()
        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = st, "map", "base_footprint"
        gt.pose.pose.position.x, gt.pose.pose.position.y = x, y
        q = yaw_quat(yaw)
        gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w = q
        gt.twist.twist.linear.x, gt.twist.twist.angular.z = v, w
        self.pub_gt.publish(gt)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = st, "imu_link"
        d = self.sim.imu()
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(a) for a in d["gyro"])
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(a) for a in d["accel"])
        qw, qx, qy, qz = (float(a) for a in d["quat"])
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = qx, qy, qz, qw
        self.pub_imu.publish(imu)
        if self.backend != "mujoco":
            return
        al, ar = self.sim.wheel_angles()
        ox, oy, oyaw = self.odom.update(al, ar, 0.02)
        od = Odometry()
        od.header.stamp, od.header.frame_id, od.child_frame_id = st, "odom", "base_footprint"
        od.pose.pose.position.x, od.pose.pose.position.y = ox, oy
        q = yaw_quat(oyaw)
        od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w = q
        od.twist.twist.linear.x, od.twist.twist.angular.z = self.odom.v, self.odom.w
        od.pose.covariance[0] = od.pose.covariance[7] = 1e-3
        od.pose.covariance[35] = 1e-2
        self.pub_odom.publish(od)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = st, "odom", "base_footprint"
        tf.transform.translation.x, tf.transform.translation.y = ox, oy
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tfb.sendTransform(tf)
        js = JointState()
        js.header.stamp = st
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [al, ar]
        js.velocity = list(self.sim.wheel_velocities())
        self.pub_js.publish(js)

    def _publish_scan(self, t):
        with self.lock:
            angles, ranges = self.sim.lidar()
        sc = LaserScan()
        sc.header.stamp, sc.header.frame_id = stamp(t), "laser_frame"
        sc.angle_min, sc.angle_max = float(angles[0]), float(angles[-1])
        sc.angle_increment = float(angles[1] - angles[0])
        sc.time_increment = 0.0
        sc.scan_time = 1.0 / self.lidar_rate
        sc.range_min, sc.range_max = 0.15, 12.0
        sc.ranges = [float(r) if np.isfinite(r) else float("inf") for r in ranges]
        self.pub_scan.publish(sc)

    def _publish_slow(self, t):
        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = stamp(t), "map"
        for cx, cy, r in self.sim.people_circles() + list(self.sim.extra_obstacles):
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = float(cx), float(cy), float(r)
            pa.poses.append(ps)
        self.pub_people.publish(pa)
        if self.backend == "mujoco" and self.is_walker:
            for pub, f in ((self.pub_fl, self.forces[0]), (self.pub_fr, self.forces[1])):
                wr = WrenchStamped()
                wr.header.stamp = stamp(t)
                wr.header.frame_id = "handle_left_ft" if pub is self.pub_fl else "handle_right_ft"
                wr.wrench.force.x = float(f)
                pub.publish(wr)
            self.pub_grip.publish(Bool(data=bool(self.grip)))

    # ------------------------------------------------------------------ viewer: MR overlay + keys
    def _style_markers(self):
        st = self.status
        for mid in self.sim.marker_mocap:
            goal = st is not None and st.goal_id == mid
            queued = st is not None and mid in list(st.queue)
            ring = [0.35, 0.95, 1.0, 0.95] if goal else ([1, 1, 1, 0.6] if queued else [0, 0, 0, 0])
            self.sim.style_marker(mid, [1, 1, 1, 0.45 + 0.25 * math.sin(self.sim.t * math.pi + mid)], ring)
        if self.markers is not None:   # markers placed in the headset move the overlay panels
            for mk in self.markers.markers:
                if mk.id in self.sim.marker_mocap:
                    mocap = self.sim.marker_mocap[mk.id][0]
                    self.sim.d.mocap_pos[mocap] = [mk.position.x, mk.position.y, mk.position.z]

    def on_key(self, k):
        ch = chr(k) if 32 <= k < 127 else ""
        if ch in "12345":
            self.pub_gaze.publish(Int32(data=int(ch)))
        elif ch.upper() == "C":
            self.pub_gaze.publish(Int32(data=6))
        elif ch == "0":
            self.pub_gaze.publish(Int32(data=0))
        elif k == 32:
            self.on_estop_button(Bool(data=not self.estop_button))
        elif ch.upper() == "R":
            self.on_estop_button(Bool(data=False))
            self.pub_estop.publish(Bool(data=False))
        elif ch.upper() == "G":
            self.grip = not self.grip
        elif ch.upper() == "W":
            self.forces = (15.0, 15.0) if self.forces[0] <= 0 else (0.0, 0.0)
        elif ch.upper() == "S":
            self.forces = (-20.0, -20.0) if self.forces[0] >= 0 else (0.0, 0.0)
        elif k in (262, 263, 264, 265):
            tw = Twist()
            tw.linear.x = {265: 0.3, 264: -0.2}.get(k, 0.0)
            tw.angular.z = {263: 0.8, 262: -0.8}.get(k, 0.0)
            self.pub_teleop.publish(tw)


def main():
    rclpy.init()
    n = MujocoBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.running = False
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

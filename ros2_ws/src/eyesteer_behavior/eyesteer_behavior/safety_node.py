"""safety_node: the ROS-side safety chain around twist_mux and the Nav2 collision monitor.

  cmd_vel_nav_raw --(walker pull-back scale)--> cmd_vel_nav  -> twist_mux -> cmd_vel_mux
  cmd_vel_mux -> [Nav2 collision_monitor] -> cmd_vel_cm
  cmd_vel_cm --(rotation guard, staleness watchdog D9)--> /cmd_vel  (consumed by the backend)

Locks for twist_mux: /lock/estop (latched until /system/reset), /lock/deadman (walker grip released
> 0.3 s or grip data stale). Also publishes the active mux source (/mux/source) and the collision
monitor state (/collision_monitor/state) for /robot/status, computed with eyesteer_core rules.
"""
from __future__ import annotations

import math

from eyesteer_behavior import _root  # noqa: F401

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import Buffer, TransformListener

from eyesteer_core import config
from eyesteer_core.control.footprint import Footprint, RotationGuard
from eyesteer_core.control.mux_rules import LOCKS, TOPICS, DeadMan, Mux
from eyesteer_core.control.mux_rules import Twist as CTwist


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety")
        self.robot = self.declare_parameter("robot", "field_robot").value
        self.timeout = self.declare_parameter("cmd_vel_timeout", 0.5).value
        self.is_walker = self.robot == "walker"
        r = config.robot(self.robot)
        self.deadman = DeadMan(r.get("deadman", {}).get("release_timeout_s", 0.3))
        self.fp = Footprint(config.geometry(self.robot)["footprint"])
        self.guard = RotationGuard(self.fp)
        self.lidar = r["lidar"]
        self.mux = Mux()
        self.scale = 1.0
        self.grip, self.grip_t = False, -1e9
        self.estop_latched = False
        self.mcu_estop = False
        self.last_cm, self.last_cm_t = Twist(), -1e9
        self.last_mux = Twist()
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.pub_nav = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.pub_out = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_estop = self.create_publisher(Bool, "/lock/estop", 10)
        self.pub_dead = self.create_publisher(Bool, "/lock/deadman", 10)
        self.pub_src = self.create_publisher(String, "/mux/source", 10)
        self.pub_cm = self.create_publisher(String, "/collision_monitor/state", 10)
        self.create_subscription(Twist, "/cmd_vel_nav_raw", self.on_nav_raw, 10)
        self.create_subscription(Float32, "/walker/nav_scale", lambda m: setattr(self, "scale", float(m.data)), 10)
        for t in TOPICS:
            self.create_subscription(Twist, "/" + t["topic"], lambda m, n=t["name"]: self.mux.input(n, CTwist(m.linear.x, m.angular.z), self.now()), 10)
        self.create_subscription(Bool, "/lock/health", lambda m: self.mux.lock("health", m.data), 10)
        self.create_subscription(Twist, "/cmd_vel_mux", lambda m: setattr(self, "last_mux", m), 10)
        self.create_subscription(Twist, "/cmd_vel_cm", self.on_cm, 10)
        self.create_subscription(Bool, "/grip", self.on_grip, 10)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)
        self.create_subscription(Bool, "/mcu/estop", lambda m: setattr(self, "mcu_estop", m.data), 10)
        self.create_subscription(Empty, "/system/reset", self.on_reset, 10)
        if self.is_walker:
            self.create_subscription(LaserScan, "/scan", self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_nav_raw(self, m: Twist):
        out = Twist()
        k = self.scale if self.is_walker else 1.0
        out.linear.x, out.angular.z = m.linear.x * k, m.angular.z * k
        self.pub_nav.publish(out)

    def on_grip(self, m: Bool):
        self.grip, self.grip_t = m.data, self.now()

    def on_estop(self, m: Bool):
        if m.data:
            self.estop_latched = True

    def on_reset(self, _):
        if not self.mcu_estop:
            self.estop_latched = False

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:
            return None

    def on_scan(self, s: LaserScan):
        p = self.pose()
        if p is None:
            return
        r = np.asarray(s.ranges, float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + self.lidar["yaw"]
        ok = np.isfinite(r) & (r > s.range_min)
        bx, by = self.lidar["x"] + r[ok] * np.cos(a[ok]), self.lidar["y"] + r[ok] * np.sin(a[ok])
        c, sn = math.cos(p[2]), math.sin(p[2])
        pts = np.stack([p[0] + c * bx - sn * by, p[1] + sn * bx + c * by], 1)
        self.guard.add_scan(self.now(), pts)

    def on_cm(self, m: Twist):
        out = m
        if self.is_walker and abs(m.angular.z) > 0.05:
            p = self.pose()
            if p is not None and self.guard.sweep_blocked(*p, m.linear.x, m.angular.z):
                out = Twist()
                out.linear.x = m.linear.x
        self.last_cm, self.last_cm_t = out, self.now()

    def tick(self):
        t = self.now()
        estop = self.estop_latched or self.mcu_estop
        self.pub_estop.publish(Bool(data=estop))
        self.mux.lock("estop", estop)
        if self.is_walker:
            gripped = self.grip and (t - self.grip_t) < 0.5
            locked = self.deadman.update(gripped, t)
            self.pub_dead.publish(Bool(data=locked))
            self.mux.lock("deadman", locked)
        _, src = self.mux.output(t)
        self.pub_src.publish(String(data=src))
        # collision-monitor state: compare its input (mux out) with its output
        vin, vout = self.last_mux.linear.x, self.last_cm.linear.x
        state = "OK"
        if vin > 0.02:
            if vout <= 0.005:
                state = "STOP"
            elif vout < 0.8 * vin:
                state = "SLOWDOWN"
        self.pub_cm.publish(String(data=state))
        # D9 watchdog: stale -> zero
        out = self.last_cm if (t - self.last_cm_t) <= self.timeout else Twist()
        self.pub_out.publish(out)


def main():
    rclpy.init()
    n = SafetyNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

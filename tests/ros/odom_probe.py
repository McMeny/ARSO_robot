#!/usr/bin/env python3
"""Drive the robot with teleop and compare /odom against /sim/ground_truth (HIL odometry check).
Run against a running sim stack:  python3 tests/ros/odom_probe.py"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from eyesteer_interfaces.msg import SystemHealth


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("odom_probe")
        self.odom = self.gt = None
        self.unlocked = False
        self.n_odom = 0
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Odometry, "/sim/ground_truth", lambda m: setattr(self, "gt", m), 10)
        self.create_subscription(SystemHealth, "/system/health", lambda m: setattr(self, "unlocked", m.motion_unlocked), 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_people = self.create_publisher(String, "/sim/people_cmd", 10)

    def on_odom(self, m):
        self.odom = m
        self.n_odom += 1

    def snap(self):
        o, g = self.odom.pose.pose, self.gt.pose.pose
        return (o.position.x, o.position.y, yaw(o.orientation)), (g.position.x, g.position.y, yaw(g.orientation))

    def spin_for(self, secs, twist=None):
        t0 = time.time()
        while time.time() - t0 < secs:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    p = Probe()
    t0 = time.time()
    while not (p.unlocked and p.odom and p.gt) and time.time() - t0 < 120:
        rclpy.spin_once(p, timeout_sec=0.2)
    print(f"unlocked={p.unlocked} odom msgs={p.n_odom}")
    for _ in range(3):   # keep the moving people out of the way of this odometry check
        p.pub_people.publish(String(data='{"disable": ["shopper_1", "shopper_2", "person_1", "person_2"]}'))
        p.spin_for(0.2)
    n0 = p.n_odom
    p.spin_for(2.0)
    print(f"/odom rate ~ {(p.n_odom - n0) / 2.0:.1f} Hz")
    for label, tw in (("forward 0.2 m/s x4 s", (0.2, 0.0)), ("turn 0.5 rad/s x4 s", (0.0, 0.5))):
        a = p.snap()
        t = Twist()
        t.linear.x, t.angular.z = tw
        p.spin_for(4.0, t)
        p.spin_for(1.0)
        b = p.snap()
        do = math.hypot(b[0][0] - a[0][0], b[0][1] - a[0][1]), b[0][2] - a[0][2]
        dg = math.hypot(b[1][0] - a[1][0], b[1][1] - a[1][1]), b[1][2] - a[1][2]
        print(f"{label}: odom dist {do[0]:.3f} m dyaw {do[1]:+.3f} | truth dist {dg[0]:.3f} m dyaw {dg[1]:+.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

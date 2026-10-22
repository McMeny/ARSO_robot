#!/usr/bin/env python3
"""Print /clock vs the header stamps of /odom, /tf (odom->base_footprint), /scan and /amcl_pose (clock-consistency check)."""
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main():
    rclpy.init()
    n = Node("stamp_probe")
    s = {}
    n.create_subscription(Clock, "/clock", lambda m: s.__setitem__("clock", m.clock.sec + m.clock.nanosec * 1e-9), 10)
    n.create_subscription(Odometry, "/odom", lambda m: s.__setitem__("odom", m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), 10)
    n.create_subscription(LaserScan, "/scan", lambda m: s.__setitem__("scan", m.header.stamp.sec + m.header.stamp.nanosec * 1e-9), qos_profile_sensor_data)

    def on_tf(m):
        for t in m.transforms:
            s[f"tf {t.header.frame_id}->{t.child_frame_id}"] = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
    n.create_subscription(TFMessage, "/tf", on_tf, 50)
    t0 = time.time()
    while time.time() - t0 < 60 and len(s) < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    t1 = time.time()
    while time.time() - t1 < 2:
        rclpy.spin_once(n, timeout_sec=0.05)
    for k, v in sorted(s.items()):
        print(f"{k:40s} {v:.3f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

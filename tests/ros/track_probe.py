#!/usr/bin/env python3
"""Every second: ground truth vs TF map->base_footprint (localization) vs /odom, while the robot moves."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    n = Node("track_probe")
    s = {}
    n.create_subscription(Odometry, "/sim/ground_truth", lambda m: s.__setitem__("gt", m.pose.pose), 10)
    n.create_subscription(Odometry, "/odom", lambda m: s.__setitem__("odom", m.pose.pose), 10)
    tf = Buffer()
    TransformListener(tf, n)
    t0 = time.time()
    last = 0
    while time.time() - t0 < 40:
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.time() - last < 1.0 or "gt" not in s or "odom" not in s:
            continue
        last = time.time()
        try:
            t = tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            m = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
        except Exception as e:
            m = (float("nan"),) * 3
        g, o = s["gt"], s["odom"]
        print(f"t={time.time() - t0:4.0f} gt=({g.position.x:6.2f},{g.position.y:6.2f},{yaw(g.orientation):5.2f}) "
              f"map=({m[0]:6.2f},{m[1]:6.2f},{m[2]:5.2f}) odom=({o.position.x:6.2f},{o.position.y:6.2f},{yaw(o.orientation):5.2f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

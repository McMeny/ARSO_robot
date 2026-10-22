#!/usr/bin/env python3
"""Select a marker by synthetic gaze, then sample every stage of the velocity chain + walker sensors."""
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

from eyesteer_interfaces.msg import RobotStatus, SystemHealth

TOPICS = ["/cmd_vel", "/cmd_vel_mux", "/cmd_vel_cm"]


def main(marker=2):
    rclpy.init()
    n = Node("chain_probe")
    s = {}
    for t in TOPICS:
        n.create_subscription(Twist, t, lambda m, t=t: s.__setitem__(t, (round(m.linear.x, 3), round(m.angular.z, 3))), 10)
    n.create_subscription(Float32, "/walker/nav_scale", lambda m: s.__setitem__("nav_scale", round(m.data, 2)), 10)
    n.create_subscription(WrenchStamped, "/handle_force/left", lambda m: s.__setitem__("F_left", m.wrench.force.x), 10)
    n.create_subscription(Bool, "/grip", lambda m: s.__setitem__("grip", m.data), 10)
    n.create_subscription(String, "/mux/source", lambda m: s.__setitem__("mux", m.data), 10)
    n.create_subscription(String, "/collision_monitor/state", lambda m: s.__setitem__("cm", m.data), 10)
    n.create_subscription(Bool, "/lock/deadman", lambda m: s.__setitem__("deadman", m.data), 10)
    n.create_subscription(RobotStatus, "/robot/status", lambda m: s.__setitem__("state", m.display), 10)
    n.create_subscription(Odometry, "/sim/ground_truth", lambda m: s.__setitem__("gt", (round(m.pose.pose.position.x, 2), round(m.pose.pose.position.y, 2))), 10)
    n.create_subscription(Odometry, "/odom", lambda m: s.__setitem__("odom", (round(m.pose.pose.position.x, 2), round(m.pose.pose.position.y, 2), round(m.twist.twist.linear.x, 2))), 10)
    n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: s.__setitem__("amcl", (round(m.pose.pose.position.x, 2), round(m.pose.pose.position.y, 2))), rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    unlocked = {"v": False}
    n.create_subscription(SystemHealth, "/system/health", lambda m: unlocked.__setitem__("v", m.motion_unlocked), 10)
    gaze = n.create_publisher(Int32, "/bci/gaze", 10)
    people = n.create_publisher(String, "/sim/people_cmd", 10)
    t0 = time.time()
    while not unlocked["v"] and time.time() - t0 < 150:
        rclpy.spin_once(n, timeout_sec=0.2)
    if marker:
        people.publish(String(data='{"disable": ["shopper_1", "shopper_2"]}'))
    t0 = time.time()
    while time.time() - t0 < float(__import__("os").environ.get("PROBE_SECS", "45")):
        if marker:
            gaze.publish(Int32(data=marker if time.time() - t0 < 4.5 else 0))
        rclpy.spin_once(n, timeout_sec=0.05)
        if int((time.time() - t0) * 10) % 20 == 0:
            print(f"t={time.time() - t0:4.1f} " + " ".join(f"{k}={v}" for k, v in sorted(s.items())), flush=True)
            time.sleep(0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 2)

"""scan_mask_node: /scan_raw -> /scan with the robot's occluded LiDAR sectors set to +inf (walker: the
frame uprights and the user behind the handles). Same sectors as the sim (config/robot_*.yaml);
replaces laser_filters' angular-bounds filter, which is not always packaged."""
from __future__ import annotations

import math

from eyesteer_behavior import _root  # noqa: F401

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from eyesteer_core import config


class ScanMask(Node):
    def __init__(self):
        super().__init__("scan_mask")
        robot = self.declare_parameter("robot", "walker").value
        self.sectors = config.robot(robot)["lidar"].get("masked_sectors_deg") or []
        self.pub = self.create_publisher(LaserScan, "/scan", QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, "/scan_raw", self.cb, qos_profile_sensor_data)

    def cb(self, m: LaserScan):
        if self.sectors:
            r = list(m.ranges)
            for i in range(len(r)):
                deg = math.degrees(m.angle_min + i * m.angle_increment) % 360
                if any(lo <= deg <= hi for lo, hi in self.sectors):
                    r[i] = float("inf")
            m.ranges = r
        m.header.frame_id = "laser_frame"
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ScanMask()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

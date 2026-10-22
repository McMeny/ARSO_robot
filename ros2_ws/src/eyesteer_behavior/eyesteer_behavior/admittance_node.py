"""admittance_node (walker): handle forces -> cmd_vel_admittance in ASSIST (idle); in ENGAGED the
pull-back scale on /walker/nav_scale lets the user slow/stop navigation. The user always wins."""
from __future__ import annotations

from eyesteer_behavior import _root  # noqa: F401

import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Float32

from eyesteer_core import config
from eyesteer_core.control.admittance import Admittance, pullback_scale

from eyesteer_interfaces.msg import RobotStatus


class AdmittanceNode(Node):
    def __init__(self):
        super().__init__("admittance")
        r = config.robot("walker")
        a = r["admittance"]
        self.a = a
        self.adm = Admittance(a["mass"], a["damping"], a["yaw_inertia"], a["yaw_damping"], a["force_deadband"],
                              r["handles"]["y"], r["limits"]["v_max"], r["limits"]["w_max"])
        self.fl = self.fr = 0.0
        self.state = "IDLE"
        self.pub = self.create_publisher(Twist, "/cmd_vel_admittance", 10)
        self.pub_scale = self.create_publisher(Float32, "/walker/nav_scale", 10)
        self.create_subscription(WrenchStamped, "/handle_force/left", lambda m: setattr(self, "fl", m.wrench.force.x), 10)
        self.create_subscription(WrenchStamped, "/handle_force/right", lambda m: setattr(self, "fr", m.wrench.force.x), 10)
        self.create_subscription(RobotStatus, "/robot/status", lambda m: setattr(self, "state", m.state), 10)
        self.dt = 0.05
        self.create_timer(self.dt, self.tick)

    def tick(self):
        self.pub_scale.publish(Float32(data=float(pullback_scale(self.fl, self.fr, self.a["pullback_threshold"],
                                                                  self.a["pullback_stop"]))))
        if self.state != "IDLE":
            self.adm.reset()
            return
        v, w = self.adm.step(self.fl, self.fr, self.dt)
        if self.adm.active(self.fl, self.fr):    # publish only while pushed: otherwise the mux falls through
            t = Twist()
            t.linear.x, t.angular.z = v, w
            self.pub.publish(t)


def main():
    rclpy.init()
    n = AdmittanceNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()

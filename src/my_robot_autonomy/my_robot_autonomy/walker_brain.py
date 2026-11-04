"""Decides how cautiously the walker handles the hazards ahead, and whether it moves at all.

Speed - the lowest limit always wins:  actual = lowest of (grip, hazard cap, collision monitor)
  grip        - /walker_enabled (the VR build's stand-in for the grip lever; the operator
                presses Start / Stop walking). Released -> the walker is held at zero.
  hazard cap  - 100 % normally; careful_percent while any MISSED hazard is ahead.
                Sent to Nav2 on /speed_limit. The EEG can only ever lower the ceiling.

Wider berth - while a MISSED hazard is ahead, the costmaps' inflation is widened
  (inflation_radius / cost_scaling_factor), so Nav2 plans further away from obstacles.

Stop when commands stop - publishes zeros on cmd_vel_idle (the lowest twist_mux input),
  so if Nav2 or teleop go silent the walker gets 0 instead of carrying on at its last speed.
  While the grip is released it publishes zeros on cmd_vel_stop (above every other input).

Publishes /walker_status (JSON) and a floating state label for RViz.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from walker_interfaces.msg import Hazard, HazardArray

COSTMAPS = ['/local_costmap/local_costmap', '/global_costmap/global_costmap']


def double_param(name, value):
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))


class WalkerBrain(Node):
    def __init__(self):
        super().__init__('walker_brain')
        p = self.declare_parameter
        self.careful_percent = p('careful_percent', 40.0).value
        self.extra_clearance = p('extra_clearance', 0.35).value
        self.normal_inflation = p('normal_inflation_radius', 0.55).value
        self.normal_scaling = p('normal_cost_scaling', 3.0).value
        self.wide_scaling = p('wide_cost_scaling', 1.5).value
        self.enabled = p('start_enabled', True).value

        self.hazards = []
        self.authority = 1.0
        self.berth_wide = None   # unknown until first set
        self.berth_pending = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.stop_pub = self.create_publisher(Twist, 'cmd_vel_stop', 10)
        self.idle_pub = self.create_publisher(Twist, 'cmd_vel_idle', 10)
        self.status_pub = self.create_publisher(String, 'walker_status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, 'walker_markers', 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(Bool, 'walker_enabled', self.on_enabled, latched)
        self.create_subscription(Float32, 'machine_authority', lambda m: setattr(self, 'authority', m.data), 10)
        self.param_clients = {c: self.create_client(SetParameters, c + '/set_parameters') for c in COSTMAPS}
        self.create_timer(0.1, self.update)
        self.get_logger().info('Walker ' + ('enabled' if self.enabled else 'waiting for Start walking'))

    def on_enabled(self, msg):
        if msg.data != self.enabled:
            self.get_logger().info('Start walking' if msg.data else 'Stop walking')
        self.enabled = msg.data

    def set_berth(self, wide):
        """Widen (or restore) the costmaps' inflation - the 'wider berth' for a missed hazard."""
        if wide == self.berth_wide:
            return
        radius = self.normal_inflation + (self.extra_clearance if wide else 0.0)
        scaling = self.wide_scaling if wide else self.normal_scaling
        ready = all(c.service_is_ready() for c in self.param_clients.values())
        if not ready:
            return  # Nav2 not up yet - try again next cycle
        for client in self.param_clients.values():
            req = SetParameters.Request(parameters=[
                double_param('inflation_layer.inflation_radius', radius),
                double_param('inflation_layer.cost_scaling_factor', scaling)])
            client.call_async(req)
        self.berth_wide = wide
        self.get_logger().info(f'Berth {"WIDENED" if wide else "normal"}: inflation {radius:.2f} m')

    def update(self):
        missed_ahead = [h for h in self.hazards if h.state == Hazard.MISSED and h.path_distance > -0.3]
        escalated = bool(missed_ahead)

        if not self.enabled:
            state = 'STOPPED'
        elif escalated:
            state = 'CAREFUL'
        elif any(h.state == Hazard.PROBING for h in self.hazards):
            state = 'PROBING'
        else:
            state = 'NORMAL'
        cap = self.careful_percent if escalated else 100.0

        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        limit.speed_limit = cap
        self.speed_pub.publish(limit)
        self.idle_pub.publish(Twist())
        if not self.enabled:
            self.stop_pub.publish(Twist())
        self.set_berth(escalated)

        status = {
            'state': state,
            'enabled': self.enabled,
            'speed_cap_percent': cap,
            'extra_clearance': self.extra_clearance if escalated else 0.0,
            'missed_hazards': [h.id for h in missed_ahead],
            'machine_authority': round(self.authority, 2),
        }
        self.status_pub.publish(String(data=json.dumps(status)))

        label = Marker()
        label.header.frame_id = 'base_footprint'
        label.ns, label.id = 'walker_state', 0
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z = 1.0
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'{state}  {cap:.0f}%  head {100 * (1 - self.authority):.0f}%'
        self.marker_pub.publish(MarkerArray(markers=[label]))


def main():
    rclpy.init()
    node = WalkerBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Decides how fast Nav2 may drive, from the lidar and the human cross-check.

/obstacle_ahead (Bool) + /human_aware (Bool)  ->  /speed_limit (nav2_msgs/SpeedLimit)

  CRUISE         no obstacle                       -> 100 %
  NORMAL_AVOID   obstacle, human aware of it       -> 100 %
  CAREFUL_AVOID  obstacle, human NOT aware of it   -> careful_percent (30 %)

Back to CRUISE only after the path has been clear for clear_time (1 s), so it
doesn't flicker. If /human_aware stops arriving, the human counts as not aware.
"""
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

CRUISE, NORMAL_AVOID, CAREFUL_AVOID = 'CRUISE', 'NORMAL_AVOID', 'CAREFUL_AVOID'


class WalkerBrain(Node):
    def __init__(self):
        super().__init__('walker_brain')
        self.careful_percent = self.declare_parameter('careful_percent', 30.0).value
        self.clear_time = self.declare_parameter('clear_time', 1.0).value
        self.human_timeout = self.declare_parameter('human_timeout', 1.0).value

        self.state = CRUISE
        self.obstacle = False
        self.last_obstacle_time = None
        self.human_aware = False
        self.last_human_time = None

        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.state_pub = self.create_publisher(String, 'walker_state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'walker_markers', 10)
        self.create_subscription(Bool, 'obstacle_ahead', self.on_obstacle, 10)
        self.create_subscription(Bool, 'human_aware', self.on_human, 10)
        self.create_timer(0.1, self.update)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_obstacle(self, msg):
        self.obstacle = msg.data
        if msg.data:
            self.last_obstacle_time = self.now()

    def on_human(self, msg):
        self.human_aware = msg.data
        self.last_human_time = self.now()

    def update(self):
        now = self.now()
        human_ok = (self.human_aware and self.last_human_time is not None
                    and now - self.last_human_time < self.human_timeout)
        recently_blocked = (self.last_obstacle_time is not None
                            and now - self.last_obstacle_time < self.clear_time)

        if self.obstacle or recently_blocked:
            new_state = NORMAL_AVOID if human_ok else CAREFUL_AVOID
        else:
            new_state = CRUISE

        if new_state != self.state:
            self.get_logger().info(f'{self.state} -> {new_state}')
            self.state = new_state

        # sent every cycle, so a restarted Nav2 still gets the current limit
        limit = SpeedLimit()
        limit.header.stamp = self.get_clock().now().to_msg()
        limit.percentage = True
        limit.speed_limit = self.careful_percent if self.state == CAREFUL_AVOID else 100.0
        self.speed_pub.publish(limit)
        self.state_pub.publish(String(data=self.state))

        # floating label above the robot in RViz, e.g. "CAREFUL_AVOID 30%"
        label = Marker()
        label.header.frame_id = 'base_footprint'
        label.ns, label.id = 'walker_state', 0
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z = 1.0
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'{self.state}  {limit.speed_limit:.0f}%'
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

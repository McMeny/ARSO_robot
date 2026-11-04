"""Keyboard stand-in for the handles (and the toggle SSVEP emulator), for desktop testing.
Run in its own terminal:  ros2 run my_robot_autonomy walker_keys
(and launch with grip:=keyboard). A terminal can't see keys being held, so keys toggle:

  w      both hands on the handles (WALK)            l / r   left / right hand on or off
  s      squeeze firmly with both hands (STEER)       space   let go of both (STOP)
  e      the user noticed the nearest tagged hazard (ssvep_emulator in toggle mode)
  a / d  turn your "head" 30 deg left / right (no headset)     f  head straight ahead again
  q      quit
"""
import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener, TransformException

from walker_interfaces.msg import GripState


class WalkerKeys(Node):
    def __init__(self):
        super().__init__('walker_keys')
        self.left = self.right = 0.0
        self.head_offset = None      # None = no fake head pose
        self.grip_pub = self.create_publisher(GripState, 'walker/grip_keys', 10)
        self.toggle_pub = self.create_publisher(Empty, 'ssvep_emulator/toggle', 10)
        self.head_pub = self.create_publisher(PoseStamped, 'head_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.02, self.publish)
        self.create_timer(0.05, self.publish_head)

    def key(self, c):
        if c == 'w':
            self.left = self.right = 0.4
        elif c == 's':
            self.left = self.right = 0.9 if self.left < 0.9 else 0.4
        elif c == ' ':
            self.left = self.right = 0.0
        elif c == 'l':
            self.left = 0.0 if self.left > 0 else 0.4
        elif c == 'r':
            self.right = 0.0 if self.right > 0 else 0.4
        elif c == 'e':
            self.toggle_pub.publish(Empty())
        elif c in 'ad':
            self.head_offset = (self.head_offset or 0.0) + math.radians(30 if c == 'a' else -30)
        elif c == 'f':
            self.head_offset = 0.0
        state = 'STEER' if min(self.left, self.right) >= 0.9 else 'WALK' if min(self.left, self.right) > 0 else 'STOP'
        head = '' if self.head_offset is None else f'   head {math.degrees(self.head_offset):+.0f} deg'
        print(f'\rleft {self.left:.1f}  right {self.right:.1f}  -> {state}{head}        ', end='', flush=True)

    def publish(self):
        msg = GripState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.left, msg.right = self.left, self.right
        self.grip_pub.publish(msg)

    def publish_head(self):
        if self.head_offset is None:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if not hasattr(self, 'head_yaw'):
            self.head_yaw = yaw
        target = self.head_yaw_base(yaw) + self.head_offset
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x = t.transform.translation.x - 0.5 * math.cos(yaw)
        ps.pose.position.y = t.transform.translation.y - 0.5 * math.sin(yaw)
        ps.pose.position.z = 1.6
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(target / 2), math.cos(target / 2)
        self.head_pub.publish(ps)

    def head_yaw_base(self, yaw):
        """The head turns relative to the room, not the walker: a/d set it, 'f' resets to the walker."""
        if self.head_offset == 0.0:
            self.head_yaw = yaw
        return self.head_yaw


def main():
    rclpy.init()
    node = WalkerKeys()
    print(__doc__)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.key('')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c == 'q':
                    break
                node.key(c)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        print()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

"""Keyboard stand-in for the handles (grip and push), for desktop testing.
Run in its own terminal:  ros2 run my_robot_autonomy walker_keys
(and launch with grip:=keyboard). A terminal can't see keys being held, so keys set levels:

  space  hands on / off the handles (the dead-man grip)
  w / s  push harder / softer (pull back below zero)        x  stop pushing
  a / d  push the right / left handle harder (turn left / right)
  e      the user noticed the nearest tagged hazard (ssvep_emulator in toggle mode)
  q      quit
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from walker_interfaces.msg import GripState, HandleForce

PUSH_STEP = 4.0     # N per handle per key press
TURN_STEP = 3.0


class WalkerKeys(Node):
    def __init__(self):
        super().__init__('walker_keys')
        self.hands = False
        self.push = 0.0     # N on each handle
        self.turn = 0.0     # N more on the right handle (and less on the left)
        self.grip_pub = self.create_publisher(GripState, 'walker/grip_keys', 10)
        self.force_pub = self.create_publisher(HandleForce, 'walker/handle_force', 10)
        self.toggle_pub = self.create_publisher(Empty, 'ssvep_emulator/toggle', 10)
        self.create_timer(0.02, self.publish)

    def key(self, c):
        if c == ' ':
            self.hands = not self.hands
        elif c == 'w':
            self.push = min(self.push + PUSH_STEP, 24.0)
        elif c == 's':
            self.push = max(self.push - PUSH_STEP, -16.0)
        elif c == 'x':
            self.push = self.turn = 0.0
        elif c == 'a':
            self.turn = min(self.turn + TURN_STEP, 15.0)
        elif c == 'd':
            self.turn = max(self.turn - TURN_STEP, -15.0)
        elif c == 'e':
            self.toggle_pub.publish(Empty())
        left, right = self.forces()
        print(f'\rhands {"ON " if self.hands else "off"}   push left {left:+5.1f} N  right {right:+5.1f} N        ',
              end='', flush=True)

    def forces(self):
        return (self.push - self.turn, self.push + self.turn) if self.hands else (0.0, 0.0)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        g = GripState()
        g.header.stamp = stamp
        g.left = g.right = 0.4 if self.hands else 0.0
        self.grip_pub.publish(g)
        f = HandleForce()
        f.header.stamp = stamp
        f.left, f.right = self.forces()
        self.force_pub.publish(f)


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

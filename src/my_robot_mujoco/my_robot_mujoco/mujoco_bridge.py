"""
MuJoCo <-> ROS 2 bridge for my_robot.

  /cmd_vel (Twist)  ->  left/right wheel velocity actuators in MuJoCo
  MuJoCo joints     ->  /joint_states  (drive wheels AND casters)
  MuJoCo chassis    ->  /odom + TF odom -> base_footprint

Run robot_state_publisher alongside this (for the rest of the TF tree),
but NOT ros2_control_node: both would publish /joint_states and odom.
"""
import math
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

DEFAULT_MJCF = '/mnt/c/Users/minua/Downloads/SUMMER INTERNSHIP/eye2steering/robot_mujoco/scene.xml'

CHASSIS_BODY = 'part4__cut_extrude1'
JOINT_NAMES = [
    'left_wheel', 'right_wheel',
    'caster_left_swivel_continuous', 'caster_left_wheel',
    'caster_right_swivel_continuous', 'caster_right_wheel',
]
# Where the chassis sits relative to base_footprint (same numbers as base_footprint_joint in my_robot.urdf.xacro)
CHASSIS_IN_FOOTPRINT = np.array([0.197626, -0.008167, 0.141708])


class MujocoBridge(Node):

    def __init__(self):
        super().__init__('mujoco_bridge')

        self.declare_parameter('mjcf_path', DEFAULT_MJCF)
        self.declare_parameter('wheel_separation', 0.39)
        self.declare_parameter('wheel_radius', 0.05)
        # The right wheel's joint axis points the opposite way in the MuJoCo export,
        # so a positive speed on it drives backwards. Flip it here.
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', -1.0)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('cmd_timeout', 0.5)    # s; stop if teleop goes quiet

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.left_sign = self.get_parameter('left_wheel_sign').value
        self.right_sign = self.get_parameter('right_wheel_sign').value
        self.dt = 1.0 / self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Load the simulation
        mjcf = self.get_parameter('mjcf_path').value
        self.model = mujoco.MjModel.from_xml_path(mjcf)
        self.data = mujoco.MjData(self.model)
        self.get_logger().info(f'Loaded MuJoCo model: {mjcf}')

        # Look up indices once so the loop doesn't search by name every step
        self.chassis_id = self.model.body(CHASSIS_BODY).id
        self.left_act = self.model.actuator('left_wheel').id
        self.right_act = self.model.actuator('right_wheel').id
        self.qpos_adr = [self.model.joint(n).qposadr[0] for n in JOINT_NAMES]
        self.qvel_adr = [self.model.joint(n).dofadr[0] for n in JOINT_NAMES]
        # Report wheels in the URDF's convention (positive = forward), not MuJoCo's raw axis
        self.joint_signs = [self.left_sign, self.right_sign] + [1.0] * (len(JOINT_NAMES) - 2)
        self.free_qvel = self.model.joint(f'{CHASSIS_BODY}_freejoint').dofadr[0]

        # Physics steps per ROS publish (e.g. 0.02 s / 0.002 s = 10)
        self.steps_per_tick = max(1, round(self.dt / self.model.opt.timestep))

        self.cmd = Twist()
        self.last_cmd_time = 0.0
        self.odom_origin = None  # (x, y, yaw) of base_footprint at startup; odom starts at 0 here

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = TransformBroadcaster(self)

    def on_cmd_vel(self, msg):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def apply_cmd(self):
        if time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd.linear.x, self.cmd.angular.z

        # Diff-drive kinematics: robot speed -> wheel speeds (rad/s)
        left = (v - w * self.wheel_sep / 2.0) / self.wheel_rad
        right = (v + w * self.wheel_sep / 2.0) / self.wheel_rad
        self.data.ctrl[self.left_act] = self.left_sign * left
        self.data.ctrl[self.right_act] = self.right_sign * right

    def step(self):
        self.apply_cmd()
        for _ in range(self.steps_per_tick):
            mujoco.mj_step(self.model, self.data)

    def publish(self):
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = JOINT_NAMES
        js.position = [float(s * self.data.qpos[a]) for s, a in zip(self.joint_signs, self.qpos_adr)]
        js.velocity = [float(s * self.data.qvel[a]) for s, a in zip(self.joint_signs, self.qvel_adr)]
        self.joint_pub.publish(js)

        # base_footprint pose: chassis pose minus the chassis offset, flattened onto the floor.
        # This is the simulator's ground truth, so it never drifts (unlike wheel odometry).
        p = self.data.xpos[self.chassis_id]
        rot = self.data.xmat[self.chassis_id].reshape(3, 3)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        fp = p - rot @ CHASSIS_IN_FOOTPRINT

        # Velocity of the footprint point, expressed in base_footprint's own frame
        lin_world = self.data.qvel[self.free_qvel:self.free_qvel + 3]
        ang_world = rot @ self.data.qvel[self.free_qvel + 3:self.free_qvel + 6]
        fp_vel = lin_world + np.cross(ang_world, fp - p)
        vx = c * fp_vel[0] + s * fp_vel[1]
        vy = -s * fp_vel[0] + c * fp_vel[1]

        # Express the pose relative to where the robot started, so odom begins at (0, 0, 0)
        if self.odom_origin is None:
            self.odom_origin = (fp[0], fp[1], yaw)
        x0, y0, yaw0 = self.odom_origin
        dx, dy = fp[0] - x0, fp[1] - y0
        odom_x = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        odom_y = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        odom_yaw = yaw - yaw0

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(odom_x)
        odom.pose.pose.position.y = float(odom_y)
        odom.pose.pose.orientation.z = math.sin(odom_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(odom_yaw / 2.0)
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.angular.z = float(ang_world[2])
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(tf)


def main():
    rclpy.init()
    node = MujocoBridge()

    # Open the MuJoCo viewer window; the loop below drives both physics and ROS.
    with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
        while rclpy.ok() and viewer.is_running():
            tick_start = time.monotonic()

            rclpy.spin_once(node, timeout_sec=0.0)  # handle any new /cmd_vel
            node.step()
            node.publish()
            viewer.sync()

            # Keep the simulation running at real-time speed
            remaining = node.dt - (time.monotonic() - tick_start)
            if remaining > 0:
                time.sleep(remaining)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

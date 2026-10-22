"""View a robot model in RViz: ros2 launch eyesteer_description view.launch.py robot:=walker"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory("eyesteer_description")
    robot = LaunchConfiguration("robot").perform(context)
    desc = xacro.process_file(os.path.join(share, "urdf", f"{robot}.urdf.xacro"), mappings={"backend": "mujoco"}).toxml()
    return [Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": desc}]),
            Node(package="joint_state_publisher", executable="joint_state_publisher"),
            Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "eyesteer.rviz")])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("robot", default_value="field_robot"), OpaqueFunction(function=setup)])

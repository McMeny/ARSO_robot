"""Real robot on the Jetson: the identical upper stack with real drivers.

ros2 launch eyesteer_bringup real.launch.py robot:=walker eeg:=lsl|synthetic|replay map:=/path/to/map.yaml
  base (config/hardware.yaml base.type):
    custom_arduino_diffdrive  mcu_bridge (/dev/eyesteer_mcu) + ros2_control + diffdrive_arduino
    cmd_vel_base              the vendor driver provides /odom + odom->base_footprint and consumes /cmd_vel
  LiDAR: sllidar_ros2 (/dev/lidar) -> /scan_raw -> scan_mask -> /scan (frame laser_frame)
"""
import os

import sys

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {"robot": "", "eeg": "lsl", "map": "", "world": "demo", "rviz": "false", "slam": "false",
        "require_headset": "true", "record": "false", "user": "default", "replay_file": "", "start_pose": "0,0,0"}


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    bring = get_package_share_directory("eyesteer_bringup")
    desc = get_package_share_directory("eyesteer_description")
    root = open(os.path.join(bring, "eyesteer_root.txt")).read().strip()
    env = {"EYESTEER_ROOT": root}
    hw = yaml.safe_load(open(os.path.join(root, "config", "hardware.yaml")))
    robot = a["robot"] or hw["robot"]
    ros_link = "/tmp/eyesteer_mcu_ros"
    urdf = xacro.process_file(os.path.join(desc, "urdf", f"{robot}.urdf.xacro"),
                              mappings={"backend": "real", "device": ros_link}).toxml()
    st = {"use_sim_time": False}
    acts = [Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[st, {"robot_description": urdf}])]
    if hw["base"]["type"] == "custom_arduino_diffdrive":
        ctrl = os.path.join(root, "ros2_ws", "src", "eyesteer_bringup", "config", "generated", f"{robot}_controllers.yaml")
        sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
        from hw_interface import hw_interface_actions
        acts += hw_interface_actions(urdf, ctrl, hw["mcu"]["device"], hw["mcu"]["baud"], ros_link, robot == "walker",
                                     False, env, 2.0)
    lid = hw["lidar"]
    acts += [
        Node(package="sllidar_ros2", executable="sllidar_node", name="sllidar_node", output="screen",
             parameters=[{"serial_port": lid["device"], "serial_baudrate": lid["baud"][lid["model"]],
                          "frame_id": "laser_frame", "angle_compensate": lid["angle_compensate"],
                          "scan_mode": "Standard" if lid["model"] != "rplidar_c1" else "Standard"}],
             remappings=[("scan", "scan_raw")]),
        Node(package="eyesteer_behavior", executable="scan_mask", parameters=[{"robot": robot}], additional_env=env),
    ]
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bring, "launch", "stack.launch.py")),
        launch_arguments={"robot": robot, "world": a["world"], "map": a["map"], "eeg": a["eeg"], "use_sim_time": "false",
                          "nav_variant": "real", "mode": "real", "slam": a["slam"], "start_pose": a["start_pose"],
                          "require_headset": a["require_headset"], "record": a["record"], "user": a["user"],
                          "replay_file": a["replay_file"]}.items())
    acts.append(TimerAction(period=6.0, actions=[stack]))
    if a["rviz"] == "true":
        acts.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "eyesteer.rviz")]))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=setup)])

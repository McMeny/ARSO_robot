"""Simulation: MuJoCo is just another hardware backend for the same stack (D15, D20).

ros2 launch eyesteer_bringup sim.launch.py world:=field|supermarket robot:=field_robot|walker \\
    backend:=mujoco|mujoco_hil eeg:=synthetic|replay|lsl demo:=true \\
    rviz:=true viewer:=true scenario:=<name> require_headset:=false record:=false

demo:=true           desktop demo without a headset or a Muse: the browser's 2D map is the "headset" (click a
                     marker or press its number to look at it), SIMULATED EEG through the real decoder, no
                     on-screen flicker; forces eeg:=synthetic (unless replay) and require_headset:=false

backend:=mujoco      mujoco_bridge publishes /odom + TF and consumes /cmd_vel (the robot "driver")
backend:=mujoco_hil  mujoco_bridge hosts the MCU emulator (firmware core on a pty) -> mcu_bridge ->
                     ros2_control + diffdrive_arduino (the real hardware-interface path) -> /odom + TF
"""
import os

import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

ARGS = {"world": "field", "robot": "field_robot", "backend": "mujoco", "eeg": "synthetic", "gaze": "headset",
        "rviz": "false", "viewer": "false", "scenario": "", "require_headset": "false", "record": "false",
        "slam": "false", "replay_file": "", "user": "default", "web": "true", "rosbridge": "true", "demo": "false"}


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    if a["demo"] == "true":                     # no headset, no Muse: see the module docstring
        a["require_headset"] = "false"
        a["eeg"] = a["eeg"] if a["eeg"] == "replay" else "synthetic"
    bring = get_package_share_directory("eyesteer_bringup")
    desc = get_package_share_directory("eyesteer_description")
    root = open(os.path.join(bring, "eyesteer_root.txt")).read().strip()
    env = {"EYESTEER_ROOT": root}
    robot, backend = a["robot"], a["backend"]
    hil = backend == "mujoco_hil"
    ros_link = "/tmp/eyesteer_mcu_ros"
    urdf = xacro.process_file(os.path.join(desc, "urdf", f"{robot}.urdf.xacro"),
                              mappings={"backend": backend, "device": ros_link}).toxml()
    st = {"use_sim_time": True}
    acts = [
        Node(package="eyesteer_mujoco", executable="mujoco_bridge", output="screen", additional_env=env,
             parameters=[st, {"world": a["world"], "robot": robot, "backend": backend, "viewer": a["viewer"] == "true",
                              "mcu_link": "/tmp/eyesteer_mcu"}]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[st, {"robot_description": urdf}]),
    ]
    if hil:
        ctrl = os.path.join(bring, "config", "generated", f"{robot}_controllers.yaml")
        if not os.path.exists(ctrl):
            ctrl = os.path.join(root, "ros2_ws", "src", "eyesteer_bringup", "config", "generated", f"{robot}_controllers.yaml")
        sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
        from hw_interface import hw_interface_actions
        acts += hw_interface_actions(urdf, ctrl, "/tmp/eyesteer_mcu", 57600, ros_link, robot == "walker", True, env, 0.2)
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bring, "launch", "stack.launch.py")),
        launch_arguments={"robot": robot, "world": a["world"], "eeg": a["eeg"], "use_sim_time": "true",
                          "nav_variant": "sim", "mode": "hil" if hil else "sim", "slam": a["slam"],
                          "require_headset": a["require_headset"], "record": a["record"], "user": a["user"],
                          "replay_file": a["replay_file"], "web": a["web"], "rosbridge": a["rosbridge"],
                          "demo": a["demo"]}.items())
    acts.append(TimerAction(period=6.0 if hil else 1.0, actions=[stack]))
    if a["rviz"] == "true":
        acts.append(Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(desc, "rviz", "eyesteer.rviz")],
                         parameters=[st]))
    if a["scenario"]:
        acts.append(Node(package="eyesteer_behavior", executable="scenario_player", output="screen", additional_env=env,
                         parameters=[st, {"scenario": a["scenario"], "prefix": "hil" if hil else "ros"}]))
    return acts


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=setup)])

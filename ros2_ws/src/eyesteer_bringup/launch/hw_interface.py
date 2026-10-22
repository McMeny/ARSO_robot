"""Shared by sim.launch.py (HIL) and real.launch.py: mcu_bridge first, then ros2_control + spawners
ONLY after mcu_bridge reports that the MCU link is up (event-gated, not timer-gated: a timer raced
the bridge's startup and diffdrive_arduino died with LibSerial::OpenFailed)."""
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch_ros.actions import Node

READY_TOKEN = "(diffdrive_arduino)"   # printed by mcu_bridge once both ends of the proxy are open


def hw_interface_actions(urdf: str, ctrl_yaml: str, device: str, baud: int, ros_link: str, walker: bool,
                         sim_time: bool, env: dict, reset_wait: float):
    st = {"use_sim_time": sim_time}
    bridge = Node(package="eyesteer_behavior", executable="mcu_bridge", output="screen", additional_env=env,
                  parameters=[st, {"device": device, "baud": baud, "ros_link": ros_link, "walker": walker,
                                   "reset_wait": reset_wait}])
    started = {"done": False}

    def on_io(event):
        if started["done"] or READY_TOKEN not in event.text.decode(errors="ignore"):
            return None
        started["done"] = True
        return [
            LogInfo(msg="[eyesteer] MCU link up -> starting ros2_control (diffdrive_arduino)"),
            Node(package="controller_manager", executable="ros2_control_node", output="screen",
                 parameters=[st, {"robot_description": urdf}, ctrl_yaml],
                 remappings=[("/diff_cont/cmd_vel_unstamped", "/cmd_vel"), ("/diff_cont/odom", "/odom")]),
            Node(package="controller_manager", executable="spawner", arguments=["joint_broad"], parameters=[st]),
            Node(package="controller_manager", executable="spawner", arguments=["diff_cont"], parameters=[st]),
        ]

    return [bridge, RegisterEventHandler(OnProcessIO(target_action=bridge, on_stdout=on_io, on_stderr=on_io))]

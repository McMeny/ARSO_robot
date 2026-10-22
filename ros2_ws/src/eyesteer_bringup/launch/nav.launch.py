"""Localization (map_server + AMCL) + Nav2 only, on top of an already running backend (sim or real),
e.g. to test Nav2 goals from RViz during bring-up:
  ros2 launch eyesteer_bringup nav.launch.py robot:=walker map:=/path/map.yaml use_sim_time:=false nav_variant:=real
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {"robot": "field_robot", "world": "field", "map": "", "use_sim_time": "true", "nav_variant": "sim", "start_pose": "0,0,0"}


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    share = get_package_share_directory("eyesteer_bringup")
    params = os.path.join(share, "params", "generated", f"{a['robot']}_nav2_{a['nav_variant']}.yaml")
    common = {"use_sim_time": a["use_sim_time"] == "true"}
    sx, sy, syaw = (float(v) for v in a["start_pose"].split(","))
    remap = [("cmd_vel", "cmd_vel_nav_raw")]
    names = ["map_server", "amcl", "controller_server", "planner_server", "behavior_server", "bt_navigator", "collision_monitor"]
    return [
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[params, common, {"yaml_filename": a["map"] or os.path.join(share, "maps", f"{a['world']}.yaml")}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl",
             parameters=[params, common, {"initial_pose.x": sx, "initial_pose.y": sy, "initial_pose.yaw": syaw}]),
        Node(package="nav2_controller", executable="controller_server", parameters=[params, common], remappings=remap),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", parameters=[params, common]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", parameters=[params, common], remappings=remap),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", parameters=[params, common]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", parameters=[params, common]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_nav",
             parameters=[common, {"autostart": True, "node_names": names, "bond_timeout": 0.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=setup)])

"""Map a space with slam_toolbox (works in sim and on the robot); save with
  ros2 run nav2_map_server map_saver_cli -f ~/maps/demo_room
Sim:  ros2 launch eyesteer_bringup slam.launch.py sim:=true world:=field robot:=field_robot
Real: ros2 launch eyesteer_bringup slam.launch.py sim:=false robot:=walker   (drive with teleop)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ARGS = {"sim": "true", "world": "field", "robot": "field_robot", "eeg": "synthetic", "backend": "mujoco"}


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    bring = get_package_share_directory("eyesteer_bringup")
    if a["sim"] == "true":
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bring, "launch", "sim.launch.py")),
                launch_arguments={"world": a["world"], "robot": a["robot"], "slam": "true", "eeg": a["eeg"],
                                  "backend": a["backend"]}.items())]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bring, "launch", "real.launch.py")),
            launch_arguments={"robot": a["robot"], "slam": "true", "eeg": a["eeg"], "require_headset": "false"}.items())]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=setup)])

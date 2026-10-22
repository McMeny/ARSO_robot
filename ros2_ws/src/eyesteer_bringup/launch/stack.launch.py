"""The upper stack: IDENTICAL in sim, HIL and on the real robot (D15). Included by sim/real.launch.py.

localization (map_server + AMCL on a saved map, or slam_toolbox) -> Nav2 (planner, RPP controller,
behaviours, BT navigator, collision monitor) -> twist_mux -> safety chain -> /cmd_vel
+ mode_manager, wander (field) / admittance (walker), anchor_bridge, health_monitor,
  BCI (Muse LSL emulator unless eeg:=lsl, ssvep_node), rosbridge (wss) + HTTPS web server.
"""
import os
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    "robot": "field_robot", "world": "field", "map": "", "eeg": "synthetic", "use_sim_time": "true",
    "nav_variant": "sim", "mode": "sim", "slam": "false", "require_headset": "false", "user": "default",
    "replay_file": "", "record": "false", "web": "true", "rosbridge": "true", "start_pose": "",
}


def _root():
    share = get_package_share_directory("eyesteer_bringup")
    p = os.path.join(share, "eyesteer_root.txt")
    return open(p).read().strip() if os.path.exists(p) else os.environ.get("EYESTEER_ROOT", "")


def setup(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    share = get_package_share_directory("eyesteer_bringup")
    root = _root()
    env = {"EYESTEER_ROOT": root}
    sim_time = a["use_sim_time"] == "true"
    robot, world = a["robot"], a["world"]
    params = os.path.join(share, "params", "generated", f"{robot}_nav2_{a['nav_variant']}.yaml")
    map_yaml = a["map"] or os.path.join(share, "maps", f"{world}.yaml")
    if a["start_pose"]:
        sx, sy, syaw = (float(v) for v in a["start_pose"].split(","))
    else:
        w = yaml.safe_load(open(os.path.join(root, "config", "worlds", f"{world}.yaml"))) if os.path.exists(
            os.path.join(root, "config", "worlds", f"{world}.yaml")) else {"start_pose": [0, 0, 0]}
        sx, sy, syaw = w["start_pose"]
    common = {"use_sim_time": sim_time}
    nodes = []
    # ---------------- localization
    if a["slam"] == "true":
        nodes.append(Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                          parameters=[os.path.join(share, "params", "slam.yaml"), common], output="screen"))
    else:
        nodes += [
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[params, common, {"yaml_filename": map_yaml}], output="screen"),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
                 parameters=[params, common, {"initial_pose.x": float(sx), "initial_pose.y": float(sy),
                                              "initial_pose.yaw": float(syaw), "set_initial_pose": True}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 parameters=[common, {"autostart": True, "node_names": ["map_server", "amcl"], "bond_timeout": 0.0}]),
        ]
    # ---------------- Nav2 (all velocity outputs go through twist_mux + collision monitor)
    bt = {}
    if robot == "walker":
        bt = {"default_nav_to_pose_bt_xml": os.path.join(share, "behavior_trees", "walker_nav_to_pose.xml")}
    remap_nav = [("cmd_vel", "cmd_vel_nav_raw")]
    nodes += [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params, common], remappings=remap_nav),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, common]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params, common], remappings=remap_nav),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params, common, bt]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen",
             parameters=[params, common]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[common, {"autostart": True, "bond_timeout": 0.0,
                                  "node_names": ["controller_server", "planner_server", "behavior_server",
                                                 "bt_navigator", "collision_monitor"]}]),
        Node(package="twist_mux", executable="twist_mux", name="twist_mux", output="screen",
             parameters=[os.path.join(share, "params", "generated", "twist_mux.yaml"), common],
             remappings=[("cmd_vel_out", "cmd_vel_mux")]),
    ]
    # ---------------- eyesteer behaviour + safety
    beh = "eyesteer_behavior"
    nodes += [
        Node(package=beh, executable="safety", parameters=[common, {"robot": robot}], additional_env=env, output="screen"),
        Node(package=beh, executable="mode_manager", parameters=[common, {"robot": robot}], additional_env=env, output="screen"),
        Node(package=beh, executable="anchor_bridge", additional_env=env, output="screen",
             parameters=[common, {"map_name": world, "robot": robot, "persist": a["mode"] == "real"}]),
        Node(package=beh, executable="health_monitor", additional_env=env, output="screen",
             parameters=[common, {"mode": a["mode"], "eeg": a["eeg"], "robot": robot, "slam": a["slam"] == "true",
                                  "require_headset": a["require_headset"] == "true"}]),
    ]
    if robot == "walker":
        nodes.append(Node(package=beh, executable="admittance", parameters=[common], additional_env=env, output="screen"))
    else:
        nodes.append(Node(package=beh, executable="wander", parameters=[common, {"world": world}], additional_env=env, output="screen"))
    # ---------------- BCI: synthetic/replay always go through a real LSL stream (Muse emulator)
    label = {"synthetic": "SIMULATED", "replay": "REPLAYED", "lsl": "REAL"}[a["eeg"]]
    import uuid
    emu_sid = f"eyesteer-muse-emulator-{uuid.uuid4().hex[:8]}"   # unique per launch: never decode a stray stream
    if a["eeg"] in ("synthetic", "replay"):
        nodes.append(Node(package="eyesteer_bci", executable="muse_emulator", additional_env=env, output="screen",
                          parameters=[common, {"mode": a["eeg"], "replay_file": a["replay_file"], "source_id": emu_sid}]))
    import socket
    bci_p = {"source_label": label, "user": a["user"]}
    if a["eeg"] in ("synthetic", "replay"):
        bci_p["stream_host"] = socket.gethostname()
        bci_p["stream_source_id"] = emu_sid           # our own emulator, never a stray stream on the network
    nodes.append(Node(package="eyesteer_bci", executable="ssvep_node", additional_env=env, output="screen",
                      parameters=[common, bci_p]))
    # ---------------- headset link: rosbridge (wss) + HTTPS server
    cert = os.path.join(root, "deploy", "certs", "eyesteer.pem")
    key = os.path.join(root, "deploy", "certs", "eyesteer.key")
    if a["rosbridge"] == "true":
        rb = {"port": 9090, "address": "", "use_sim_time": sim_time}
        if os.path.exists(cert):
            rb.update({"ssl": True, "certfile": cert, "keyfile": key})
        nodes.append(Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
                          parameters=[rb], output="screen", respawn=True, respawn_delay=2.0))
        nodes.append(Node(package="rosapi", executable="rosapi_node", name="rosapi", parameters=[common]))
    if a["web"] == "true":
        nodes.append(Node(package=beh, executable="web_server", additional_env=env, output="screen",
                          parameters=[{"robot": robot, "eeg": a["eeg"], "mode": a["mode"]}]))
    if a["record"] == "true":
        out = os.path.join(root, "outputs", "logs", "rosbag")
        nodes.append(ExecuteProcess(cmd=["ros2", "bag", "record", "-a", "-x", "/(clock|map_updates|.*costmap.*)", "-o",
                                         os.path.join(out, f"{robot}_{world}_{time.strftime('%Y%m%d_%H%M%S')}")], output="screen"))
        nodes.append(Node(package="eyesteer_bci", executable="lsl_recorder", additional_env=env, output="screen",
                          parameters=[{"out": os.path.join(root, "outputs", "logs", f"eeg_{robot}_{world}.xdf")}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=setup)])

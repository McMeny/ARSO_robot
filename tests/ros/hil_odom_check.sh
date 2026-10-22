#!/usr/bin/env bash
# HIL: drive straight then turn with teleop and compare /odom (diff_drive_controller via diffdrive_arduino
# + MCU emulator) against MuJoCo ground truth.
cd "$(dirname "$0")/../.." || exit 1
# shellcheck disable=SC1091
source deploy/env.sh
pkill -9 -f "ros2 launch eyesteer" 2>/dev/null; sleep 1
ros2 launch eyesteer_bringup sim.launch.py backend:=mujoco_hil web:=false rosbridge:=false > /tmp/hil_odom.log 2>&1 &
PID=$!
sleep 30
echo "--- before"; timeout 5 ros2 topic echo --once /odom --field pose.pose.position; timeout 5 ros2 topic echo --once /sim/ground_truth --field pose.pose.position
timeout 4 ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}" > /dev/null
echo "--- after 4 s forward 0.2 m/s"; timeout 5 ros2 topic echo --once /odom --field pose.pose; timeout 5 ros2 topic echo --once /sim/ground_truth --field pose.pose
timeout 4 ros2 topic pub -r 20 /cmd_vel_teleop geometry_msgs/msg/Twist "{angular: {z: 0.5}}" > /dev/null
echo "--- after 4 s turn 0.5 rad/s"; timeout 5 ros2 topic echo --once /odom --field pose.pose.orientation; timeout 5 ros2 topic echo --once /sim/ground_truth --field pose.pose.orientation
timeout 3 ros2 topic echo --once /joint_states
kill -INT $PID; sleep 3; pkill -9 -f "eyesteer_|nav2_|twist_mux|ros2_control|controller_manager|robot_state_publisher|lifecycle_manager|amcl|map_server" 2>/dev/null; true

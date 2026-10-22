#!/usr/bin/env bash
# Bring up the sim stack, wait, report topic rates / health / TF, then shut down.
# Usage: tests/ros/smoke_sim.sh <world> <robot> <backend> [seconds]
set +u
cd "$(dirname "$0")/../.."
source deploy/env.sh
WORLD=${1:-field}; ROBOT=${2:-field_robot}; BACKEND=${3:-mujoco}; SECS=${4:-45}
LOG=/tmp/eyesteer_smoke_${ROBOT}_${BACKEND}.log
ros2 launch eyesteer_bringup sim.launch.py world:=$WORLD robot:=$ROBOT backend:=$BACKEND web:=false > $LOG 2>&1 &
PID=$!
sleep $SECS
echo "=== topics"; ros2 topic list | grep -E "^/(scan|odom|clock|map|markers|robot/status|bci/status|system/health|cmd_vel|amcl_pose)$" | tr '\n' ' '; echo
echo "=== scan rate"; timeout 6 ros2 topic hz /scan 2>/dev/null | grep -m1 average
echo "=== bci rate"; timeout 6 ros2 topic echo --once /bci/status 2>/dev/null | grep -E "sample_rate|quality|source|signal_ok"
echo "=== status"; timeout 5 ros2 topic echo --once /robot/status 2>/dev/null | grep -E "display|mux_source|safety|motion_locked"
echo "=== health"; timeout 15 ros2 topic echo --once /system/health 2>/dev/null | grep -E "name|level|message|motion_unlocked|all_required" | paste - - - | head -20
echo "=== tf"; timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -m2 -E "Translation|Rotation"
echo "=== errors in log"; grep -iE "error|exception|traceback" $LOG | grep -v "Bond" | head -20
kill -INT $PID; sleep 5; kill -9 $PID 2>/dev/null; pkill -9 -f eyesteer 2>/dev/null; pkill -9 -f "ros2|nav2|rosbridge|twist_mux|_server|amcl|map_server" 2>/dev/null; true

#!/usr/bin/env bash
# Run one scenario through the full ROS sim stack (scenario_player) and print its result.
# Usage: tests/ros/run_scenario.sh <scenario> [backend=mujoco|mujoco_hil] [timeout_s]
set +u
cd "$(dirname "$0")/../.."
source deploy/env.sh
SCN=$1; BACKEND=${2:-mujoco}; TMO=${3:-400}
read -r WORLD ROBOT < <(python3 -c "import yaml;d=yaml.safe_load(open('config/scenarios/$SCN.yaml'));print(d['world'],d['robot'])")
OUT=outputs/metrics/ros_${SCN}.json
rm -f "$OUT"
LOG=outputs/logs/ros_${SCN}_${BACKEND}.log
mkdir -p outputs/logs
ros2 launch eyesteer_bringup sim.launch.py world:=$WORLD robot:=$ROBOT backend:=$BACKEND scenario:=$SCN web:=false > $LOG 2>&1 &
PID=$!
T0=$(date +%s)
while [[ ! -f "$OUT" && $(( $(date +%s) - T0 )) -lt $TMO ]]; do sleep 2; done
kill -INT $PID 2>/dev/null; sleep 4; kill -9 $PID 2>/dev/null
pkill -9 -f "eyesteer_|nav2_|rosbridge|twist_mux|ros2_control|controller_manager|robot_state_publisher|slam_toolbox|lifecycle_manager|amcl|map_server|rosapi" 2>/dev/null
sleep 1
if [[ -f "$OUT" ]]; then
  python3 - "$OUT" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
print(f"[ROS {d['scenario']}] {'PASS' if d['passed'] else 'FAIL'} sel={[s['label'] for s in d['selections']]} visited={[a['label'] for a in d['arrivals']]} states={d['states_seen']} collisions={d['collisions']} minclear={d['min_clearance_m']} mux={d['mux_share']} {d['expectation_failures']}")
sys.exit(0 if d['passed'] else 1)
PY
else
  echo "[ROS $SCN] TIMEOUT (no result after ${TMO}s); log: $LOG"; grep -iE "error|traceback" $LOG | grep -v "Timed out" | head -10; exit 1
fi

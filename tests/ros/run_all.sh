#!/usr/bin/env bash
# Run scenarios through the ROS sim stack, one after another (LSL + pty names are machine-global).
# Usage: tests/ros/run_all.sh [backend=mujoco] [scenario ...]
cd "$(dirname "$0")/../.." || exit 1
BACKEND=${1:-mujoco}; shift || true
SCNS=("$@")
[[ ${#SCNS[@]} -eq 0 ]] && SCNS=(field_basic field_queue field_no_control field_overrides supermarket_walker supermarket_queue)
FAIL=0
for s in "${SCNS[@]}"; do
  bash tests/ros/run_scenario.sh "$s" "$BACKEND" 420 2>&1 | grep -E '^\[ros_' || FAIL=1
  [[ ${PIPESTATUS[0]} -ne 0 ]] && FAIL=1
done
exit $FAIL

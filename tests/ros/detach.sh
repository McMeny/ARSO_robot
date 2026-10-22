#!/usr/bin/env bash
# Run a long command detached inside WSL/Linux (survives the calling shell): tests/ros/detach.sh <log> <cmd...>
LOG="$1"; shift
cd "$(dirname "$0")/../.." || exit 1
setsid nohup bash -c "$*; echo \"[detach] exit \$?\"" > "$LOG" 2>&1 < /dev/null &
echo "[detach] started pid $! -> $LOG"

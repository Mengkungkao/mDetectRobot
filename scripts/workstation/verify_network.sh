#!/usr/bin/env bash
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
fail=0
for topic in /scan /odom /imu/data /joint_states; do
  if timeout 8 ros2 topic echo "$topic" --once >/dev/null 2>&1; then echo "[PASS] Pi topic visible: $topic"; else echo "[FAIL] Pi topic missing: $topic"; fail=1; fi
done
timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint >/dev/null 2>&1 && echo '[PASS] robot TF visible' || { echo '[FAIL] robot TF missing'; fail=1; }
exit "$fail"

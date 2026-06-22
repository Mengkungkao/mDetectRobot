#!/usr/bin/env bash
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
fail=0
check_node() { ros2 node list 2>/dev/null | grep -qx "$1" && echo "[PASS] node $1" || { echo "[FAIL] node $1"; fail=1; }; }
check_topic() {
  local topic="$1" timeout_s="${2:-8}"
  if timeout "$timeout_s" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "[PASS] data received: $topic"
  else
    echo "[FAIL] no data: $topic"; fail=1
  fi
}
check_node /mdetect_serial_bridge
check_node /cspc_lidar
check_node /mdetect_cmd_mux
check_node /robot_state_publisher
check_topic /scan 10
check_topic /odom 8
check_topic /imu/data 8
check_topic /joint_states 8
check_topic /base/wheel_speeds_mm_s 8
check_topic /safety/front_distance 8

timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint >/dev/null 2>&1 && echo '[PASS] TF odom -> base_footprint' || { echo '[FAIL] TF odom -> base_footprint'; fail=1; }
timeout 6 ros2 run tf2_ros tf2_echo base_link laser >/dev/null 2>&1 && echo '[PASS] TF base_link -> laser' || { echo '[FAIL] TF base_link -> laser'; fail=1; }

echo 'LiDAR rate sample:'
timeout 5 ros2 topic hz /scan 2>&1 | tail -5 || true
echo 'Odometry rate sample:'
timeout 5 ros2 topic hz /odom 2>&1 | tail -5 || true
exit "$fail"

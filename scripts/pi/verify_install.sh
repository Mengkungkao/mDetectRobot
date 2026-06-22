#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"

fail=0
pass() { echo "[PASS] $*"; }
failmsg() { echo "[FAIL] $*"; fail=1; }
for package in mdetect_robot cspc_lidar; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 && pass "ROS package $package installed" || failmsg "ROS package $package missing"
done
ros2 pkg executables mdetect_robot | grep -q serial_bridge && pass 'serial_bridge executable' || failmsg 'serial_bridge missing'
ros2 pkg executables cspc_lidar | grep -q cspc_lidar && pass 'COIN-D6 SDK executable' || failmsg 'cspc_lidar executable missing'
python3 -c 'import serial, yaml' && pass 'Python serial/YAML modules' || failmsg 'Python dependencies'
ros2 launch mdetect_robot robot.launch.py --show-args >/dev/null && pass 'robot launch description loads' || failmsg 'robot launch description failed'
bash "$SCRIPT_DIR/verify_devices.sh" || fail=1
exit "$fail"

#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
LIVE=0
[[ "${1:-}" == "--live" ]] && LIVE=1

failures=0
ok() { printf '[PASS] %s\n' "$*"; }
bad() { printf '[FAIL] %s\n' "$*" >&2; failures=$((failures+1)); }

check_ubuntu_jammy && ok "Ubuntu 22.04 Jammy"
[[ -f /opt/ros/humble/setup.bash ]] && ok "ROS 2 Humble installed" || bad "ROS 2 Humble missing"
for device in /dev/ttyUSB0 /dev/ttyUSB1; do
  if [[ -c "$device" ]]; then
    [[ -r "$device" && -w "$device" ]] && ok "$device exists and is readable/writable" || bad "$device exists but permissions are insufficient"
    udevadm info -q property -n "$device" 2>/dev/null | grep -E '^(ID_VENDOR=|ID_MODEL=|ID_SERIAL=)' | sed "s#^#       #" || true
  else
    bad "$device is missing"
  fi
done
[[ -c /dev/ttyUSB0 ]] && ok "Configured LiDAR port: /dev/ttyUSB0" || true
[[ -c /dev/ttyUSB1 ]] && ok "Configured Arduino port: /dev/ttyUSB1" || true
id -nG "$USER" | tr ' ' '\n' | grep -qx dialout && ok "User is in dialout group" || bad "User is not active in dialout group; log out and back in"

if [[ -f "$MD_WS/install/setup.bash" ]]; then
  source_ros
  for package in cspc_lidar mdetect_base mdetect_description mdetect_bringup; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 && ok "ROS package $package" || bad "ROS package $package missing"
  done
  ros2 pkg executables cspc_lidar | grep -q 'cspc_lidar cspc_lidar' && ok "COIN-D6 executable installed" || bad "COIN-D6 executable missing"
  ros2 pkg executables mdetect_base | grep -q 'serial_bridge' && ok "Arduino serial bridge executable installed" || bad "Arduino serial bridge missing"
else
  bad "$MD_WS has not been built"
fi

if (( LIVE )); then
  source_ros
  for topic in /scan /odom /imu/data /joint_states /tf /robot/ready; do
    ros2 topic list | grep -qx "$topic" && ok "Live topic $topic" || bad "Live topic $topic missing"
  done
  timeout 8 ros2 topic echo /scan --once >/dev/null 2>&1 && ok "LiDAR is publishing scans" || bad "No /scan message received"
  timeout 8 ros2 topic echo /odom --once >/dev/null 2>&1 && ok "Arduino is publishing odometry" || bad "No /odom message received"
  timeout 8 ros2 topic echo /imu/data --once >/dev/null 2>&1 && ok "Arduino is publishing IMU data" || bad "No /imu/data message received"
  timeout 8 ros2 topic echo /robot/ready --once 2>/dev/null | grep -q 'data: true' && ok "Robot initializer reports ready" || bad "Robot is not ready"
fi

(( failures == 0 )) || exit 1

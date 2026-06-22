#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
NETWORK=0
[[ "${1:-}" == "--network" || "${1:-}" == "--live" ]] && NETWORK=1
failures=0
ok() { printf '[PASS] %s\n' "$*"; }
bad() { printf '[FAIL] %s\n' "$*" >&2; failures=$((failures+1)); }

check_ubuntu_jammy && ok "Ubuntu 22.04 Jammy"
[[ -f /opt/ros/humble/setup.bash ]] && ok "ROS 2 Humble installed" || bad "ROS 2 Humble missing"
source_ros
for package in mdetect_navigation mdetect_description nav2_bringup slam_toolbox rviz2; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 && ok "ROS package $package" || bad "ROS package $package missing"
done
[[ "${ROS_DOMAIN_ID:-}" == "30" ]] && ok "ROS_DOMAIN_ID=30" || bad "ROS_DOMAIN_ID must be 30"
[[ "${ROS_LOCALHOST_ONLY:-}" == "0" ]] && ok "ROS_LOCALHOST_ONLY=0" || bad "ROS_LOCALHOST_ONLY must be 0"

if (( NETWORK )); then
  for topic in /scan /odom /imu/data /tf /robot/ready; do
    ros2 topic list | grep -qx "$topic" && ok "Discovered Pi topic $topic" || bad "Cannot discover Pi topic $topic"
  done
  timeout 8 ros2 topic echo /robot/ready --once 2>/dev/null | grep -q 'data: true' && ok "Onboard robot reports ready" || bad "Onboard robot is not ready"
fi
(( failures == 0 )) || exit 1

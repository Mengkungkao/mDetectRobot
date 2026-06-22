#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

check_ubuntu_jammy
ensure_ros_apt_source
log "Installing workstation ROS 2, RViz, SLAM Toolbox and Navigation2"
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep python3-yaml rsync chrony

mkdir -p "$MD_WS/src"
for package in mdetect_description mdetect_navigation; do
  rm -rf "$MD_WS/src/$package"
  rsync -a "$PROJECT_DIR/ros2_ws/src/$package/" "$MD_WS/src/$package/"
done

source_ros
cd "$MD_WS"
colcon build --symlink-install --packages-up-to mdetect_navigation
configure_ros_environment

log "Workstation installation complete. No SHA256 checksum is required or used."
log "Use the same ROS_DOMAIN_ID=30 on the Pi and workstation."

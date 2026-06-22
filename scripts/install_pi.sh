#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"

check_ubuntu_jammy
ensure_ros_apt_source
log "Installing Raspberry Pi ROS 2 and build dependencies"
sudo apt-get install -y \
  ros-humble-ros-base \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  python3-yaml \
  build-essential cmake rsync git chrony

sudo usermod -aG dialout "$USER"
mkdir -p "$MD_WS/src"
for package in cspc_lidar mdetect_base mdetect_description mdetect_bringup; do
  rm -rf "$MD_WS/src/$package"
  rsync -a "$PROJECT_DIR/ros2_ws/src/$package/" "$MD_WS/src/$package/"
done

source_ros
log "Building onboard packages with one worker to reduce Raspberry Pi memory use"
cd "$MD_WS"
colcon build --symlink-install --parallel-workers 1 --packages-up-to mdetect_bringup
configure_ros_environment

log "Pi installation complete. No SHA256 checksum is required or used."
log "Log out and back in once so the dialout group becomes active, then run:"
printf '  %s\n' "$PROJECT_DIR/scripts/bringup_pi.sh"

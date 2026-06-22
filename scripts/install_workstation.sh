#!/usr/bin/env bash
set -euo pipefail

if [ "${ROS_DISTRO:-}" != "humble" ] && [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble is not installed. Install ROS 2 Humble first, then rerun this script."
  exit 1
fi

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rviz2

mkdir -p "$HOME/mdetect_ws/src"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cp -a "$PROJECT_ROOT/ros2_ws/src/." "$HOME/mdetect_ws/src/"

source /opt/ros/humble/setup.bash
cd "$HOME/mdetect_ws"
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

grep -q 'AutonomousV11 ROS environment' "$HOME/.bashrc" 2>/dev/null || cat >> "$HOME/.bashrc" <<'BASHRC'

# AutonomousV11 ROS environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source "$HOME/mdetect_ws/install/setup.bash"
BASHRC

echo "Workstation installation complete. Open a new terminal before running RViz2."

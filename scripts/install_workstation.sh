#!/usr/bin/env bash
set -euo pipefail

# Source a ROS/ament setup file without allowing `set -u` to break on
# optional environment variables such as AMENT_TRACE_SETUP_FILES.
safe_source_setup() {
  local setup_file="$1"
  local restore_nounset=0

  if [[ "$-" == *u* ]]; then
    restore_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "$setup_file"

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}


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

safe_source_setup /opt/ros/humble/setup.bash
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

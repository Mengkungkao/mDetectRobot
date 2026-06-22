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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$HOME/mdetect_ws"
LIDAR_RULE="$PROJECT_ROOT/ros2_ws/src/cspc_lidar/sc_mini.rules"

sudo apt update
sudo apt install -y \
  build-essential \
  libpcl-dev \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-serial \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions

sudo usermod -aG dialout "$USER"

if [ -f "$LIDAR_RULE" ]; then
  sudo install -m 0644 "$LIDAR_RULE" /etc/udev/rules.d/99-cspc-coin-d6.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
else
  echo "ERROR: COIN-D6 udev rule not found at: $LIDAR_RULE"
  exit 1
fi

mkdir -p "$WORKSPACE/src"
cp -a "$PROJECT_ROOT/ros2_ws/src/." "$WORKSPACE/src/"

# Do not carry stale overlay paths into a clean package rebuild.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
safe_source_setup /opt/ros/humble/setup.bash
cd "$WORKSPACE"
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys="ament_python"
colcon build --symlink-install

if ! grep -q 'AutonomousV11 ROS environment' "$HOME/.bashrc" 2>/dev/null; then
  cat >> "$HOME/.bashrc" <<'BASHRC'

# AutonomousV11 ROS environment
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/mdetect_ws/install/setup.bash" ] && source "$HOME/mdetect_ws/install/setup.bash"
BASHRC
fi

echo
printf '%s\n' \
  "Pi installation complete." \
  "The bundled COIN-D6 cspc_lidar driver was copied and built." \
  "The USB rule was installed as /etc/udev/rules.d/99-cspc-coin-d6.rules." \
  "Unplug and reconnect the LiDAR, then check: ls -l /dev/sc_mini" \
  "Log out and back in so the dialout group applies." \
  "Edit ~/mdetect_ws/src/mdetect_base/config/base.yaml for the Arduino port if needed." \
  "LiDAR settings are in ~/mdetect_ws/src/cspc_lidar/params/cspc_lidar.yaml."

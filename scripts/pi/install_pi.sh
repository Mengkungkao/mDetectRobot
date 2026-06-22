#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${MDETECT_WS:-$HOME/mdetect_ws}"

bash "$PROJECT_ROOT/scripts/common/install_ros2_repo.sh"
sudo apt-get install -y \
  ros-humble-ros-base ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-tf2-tools ros-humble-diagnostic-msgs ros-humble-std-srvs \
  python3-colcon-common-extensions python3-rosdep python3-serial python3-yaml \
  build-essential cmake git rsync udev chrony

sudo usermod -aG dialout "$USER"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
mkdir -p "$WS/src"
rsync -a --delete "$PROJECT_ROOT/ros2_ws/src/mdetect_robot/" "$WS/src/mdetect_robot/"
rsync -a --delete "$PROJECT_ROOT/ros2_ws/src/cspc_lidar/" "$WS/src/cspc_lidar/"

set +u
source /opt/ros/humble/setup.bash
set -u
rosdep install --from-paths "$WS/src" --ignore-src -r -y --rosdistro humble
cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

cat > "$HOME/.mdetect_ros2_env" <<ENV
export MDETECT_WS="$WS"
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
ENV

grep -qF 'source "$HOME/.mdetect_ros2_env"' "$HOME/.bashrc" || \
  echo '[[ -f "$HOME/.mdetect_ros2_env" ]] && source "$HOME/.mdetect_ros2_env"' >> "$HOME/.bashrc"

if [[ -c /dev/ttyUSB0 && -c /dev/ttyUSB1 ]]; then
  bash "$PROJECT_ROOT/scripts/pi/configure_udev.sh" /dev/ttyUSB0 /dev/ttyUSB1 || true
fi

echo
printf 'Pi installation complete. Workspace: %s\n' "$WS"
echo 'Log out and back in once so the dialout group is active.'
echo 'Then run: bash scripts/pi/verify_install.sh'

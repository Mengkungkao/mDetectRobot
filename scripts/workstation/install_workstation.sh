#!/usr/bin/env bash
set -euo pipefail
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${MDETECT_WS:-$HOME/mdetect_ws}"

bash "$PROJECT_ROOT/scripts/common/install_ros2_repo.sh"
sudo apt-get install -y \
  ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools ros-humble-rqt-robot-steering \
  python3-colcon-common-extensions python3-rosdep python3-yaml \
  build-essential cmake git rsync chrony

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
mkdir -p "$WS/src"
rsync -a --delete "$PROJECT_ROOT/ros2_ws/src/mdetect_robot/" "$WS/src/mdetect_robot/"
# Keep the SDK package present because mdetect_robot declares it as a runtime dependency.
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

echo
printf 'Workstation installation complete. Workspace: %s\n' "$WS"
echo 'Use the same ROS_DOMAIN_ID=30 on the Pi and workstation.'

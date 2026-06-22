#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCHIVE="${1:-$SCRIPT_DIR/cspc_lidar_clean_humble_v4.tar.gz}"
WORKSPACE="${WORKSPACE:-$HOME/mdetect_ws}"
PACKAGE_SRC="$WORKSPACE/src/cspc_lidar"
STAMP="$(date +%Y%m%d_%H%M%S)"

if [ ! -f "$ARCHIVE" ]; then
  echo "ERROR: Driver archive not found: $ARCHIVE"
  echo "Place cspc_lidar_clean_humble_v4.tar.gz beside this script or pass its path as argument 1."
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ERROR: ROS 2 Humble is not installed at /opt/ros/humble."
  exit 1
fi

mkdir -p "$WORKSPACE/src"
if [ -d "$PACKAGE_SRC" ]; then
  mv "$PACKAGE_SRC" "${PACKAGE_SRC}.backup_${STAMP}"
  echo "Previous driver backed up to ${PACKAGE_SRC}.backup_${STAMP}"
fi

tar -xzf "$ARCHIVE" -C "$WORKSPACE/src"

sudo usermod -aG dialout "$USER"
sudo install -m 0644 "$PACKAGE_SRC/sc_mini.rules" /etc/udev/rules.d/99-cspc-coin-d6.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Start from ROS Humble only, without stale overlay paths.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
rm -rf build/cspc_lidar install/cspc_lidar
colcon build \
  --symlink-install \
  --packages-select cspc_lidar \
  --event-handlers console_direct+

set +u
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"

echo
echo "Installed executable:"
ros2 pkg executables cspc_lidar

echo
echo "COIN-D6 driver replacement complete."
echo "Reconnect the LiDAR and check: ls -l /dev/sc_mini"
echo "Log out and back in if dialout membership was newly added."

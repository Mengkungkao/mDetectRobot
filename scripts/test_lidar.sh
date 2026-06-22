#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/mdetect_ws/install/setup.bash"

echo "=== COIN-D6 serial device ==="
if [ -e /dev/sc_mini ]; then
  ls -l /dev/sc_mini
  echo "Resolved device: $(readlink -f /dev/sc_mini)"
else
  echo "/dev/sc_mini not found. Reconnect the LiDAR and check the udev rule."
fi

echo
echo "=== Driver package ==="
ros2 pkg executables cspc_lidar

echo
echo "Starting the COIN-D6 driver. Press Ctrl+C to stop."
exec ros2 launch cspc_lidar lidar_launch.py

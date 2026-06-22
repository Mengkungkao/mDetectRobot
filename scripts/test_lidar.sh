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


safe_source_setup /opt/ros/humble/setup.bash
safe_source_setup "$HOME/mdetect_ws/install/setup.bash"

echo "=== COIN-D6 serial device ==="
if [ -e /dev/sc_mini ]; then
  ls -l /dev/sc_mini
  echo "Resolved device: $(readlink -f /dev/sc_mini)"
else
  echo "ERROR: /dev/sc_mini not found."
  echo "Run this command on the Raspberry Pi with the COIN-D6 connected."
  echo "Check available ports with: ls -l /dev/ttyUSB* 2>/dev/null"
  exit 2
fi

echo
echo "=== Driver package ==="
ros2 pkg executables cspc_lidar

echo
echo "Starting the COIN-D6 driver. Press Ctrl+C to stop."
exec ros2 launch cspc_lidar lidar_launch.py

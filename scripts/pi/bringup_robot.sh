#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
LIDAR_PORT="${LIDAR_PORT:-/dev/ttyUSB0}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/ttyUSB1}"
[[ -e /dev/coin_d6 ]] && LIDAR_PORT=/dev/coin_d6
[[ -e /dev/arduino_mdetect ]] && ARDUINO_PORT=/dev/arduino_mdetect

for dev in "$LIDAR_PORT" "$ARDUINO_PORT"; do
  [[ -c "$dev" ]] || { echo "Serial device not found: $dev" >&2; exit 1; }
done

echo "Starting mDetect robot bringup"
echo "  LiDAR : $LIDAR_PORT @ 230400"
echo "  Arduino: $ARDUINO_PORT @ 500000"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
exec ros2 launch mdetect_robot robot.launch.py \
  lidar_port:="$LIDAR_PORT" arduino_port:="$ARDUINO_PORT" \
  lidar_reverse_scan:="${LIDAR_REVERSE_SCAN:-true}" \
  lidar_angle_offset_deg:="${LIDAR_ANGLE_OFFSET_DEG:-0.0}"

#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
"$SCRIPT_DIR/verify_pi.sh"
source_ros
log "Starting mDetect onboard bringup: Pi + Uno + encoders + MPU6050 + motors + COIN-D6"
exec ros2 launch mdetect_bringup robot.launch.py \
  arduino_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB0 \
  initialize:=true

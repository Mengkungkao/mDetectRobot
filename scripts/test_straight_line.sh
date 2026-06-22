#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
SPEED="${1:-0.10}"
DURATION="${2:-3}"
source_ros
printf 'WARNING: The robot will move forward at %s m/s for %s seconds. Keep it on the floor with clear space.\n' "$SPEED" "$DURATION"
read -r -p 'Press Enter to start or Ctrl+C to cancel. '
timeout "$DURATION" ros2 topic pub -r 20 /cmd_vel_manual geometry_msgs/msg/Twist \
  "{linear: {x: $SPEED, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

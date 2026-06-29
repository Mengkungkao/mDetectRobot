#!/usr/bin/env bash
set -euo pipefail
DURATION="${1:-2}"
SPEED="${2:-0.05}"

echo "Publishing forward ${SPEED} m/s for ${DURATION} seconds. Keep the robot raised for the first test."
timeout "${DURATION}s" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: ${SPEED}, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" || true
ros2 service call /base/brake std_srvs/srv/Trigger "{}" || true

#!/usr/bin/env bash
set -euo pipefail
[[ "${1:-}" == "--confirm-robot-lifted" ]] || {
  echo 'Refusing to move motors. Lift the robot so all wheels are clear.'
  echo 'Then run: bash scripts/pi/test_actuators_on_blocks.sh --confirm-robot-lifted'
  exit 2
}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
stop_robot() { ros2 topic pub --once /cmd_vel_manual geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true; }
trap stop_robot EXIT INT TERM
run_motion() {
  local label="$1" linear="$2" angular="$3"
  echo "Testing $label"
  timeout 2 ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
    "{linear: {x: $linear}, angular: {z: $angular}}" >/dev/null 2>&1 || true
  stop_robot
  sleep 1.2
}
run_motion forward 0.08 0.0
run_motion reverse -0.08 0.0
run_motion left 0.0 0.45
run_motion right 0.0 -0.45
stop_robot
echo 'Actuator sequence complete. Confirm all four wheel directions and encoder signs from /base/wheel_speeds_mm_s.'

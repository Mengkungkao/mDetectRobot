#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
usage() {
  echo 'Usage:'
  echo '  set_motor_tuning.sh trimf MOTOR SCALE'
  echo '  set_motor_tuning.sh trimr MOTOR SCALE'
  echo '  set_motor_tuning.sh pidm MOTOR KP KI KD'
  echo '  set_motor_tuning.sh heading KP MAX_DEG_S'
  echo '  set_motor_tuning.sh show'
}
case "${1:-}" in
  trimf|trimr)
    [[ $# -eq 3 ]] || { usage; exit 2; }
    cmd="${1^^},$2,$3" ;;
  pidm)
    [[ $# -eq 5 ]] || { usage; exit 2; }
    cmd="PIDM,$2,$3,$4,$5" ;;
  heading)
    [[ $# -eq 3 ]] || { usage; exit 2; }
    cmd="HEADING,$2,$3" ;;
  show) cmd='GET_CONFIG' ;;
  *) usage; exit 2 ;;
esac
ros2 topic pub --once /base/config_command std_msgs/msg/String "{data: '$cmd'}"
echo 'Arduino reply:'
timeout 3 ros2 topic echo /base/serial_rx --once || true

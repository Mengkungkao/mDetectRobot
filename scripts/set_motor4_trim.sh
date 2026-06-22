#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
FORWARD="${1:-0.78}"
REVERSE="${2:-0.66}"
source_ros
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'TRIMF,4,$FORWARD'}"
sleep 0.3
ros2 topic pub --once /base/raw_command std_msgs/msg/String "{data: 'TRIMR,4,$REVERSE'}"
printf 'Motor 4 trims sent: forward=%s reverse=%s\n' "$FORWARD" "$REVERSE"

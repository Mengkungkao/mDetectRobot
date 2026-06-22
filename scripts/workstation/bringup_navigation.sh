#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 1 ]] || { echo 'Usage: bringup_navigation.sh /absolute/path/to/map.yaml'; exit 2; }
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../common/ros_env.sh"
MAP="$(readlink -f "$1")"
[[ -f "$MAP" ]] || { echo "Map not found: $MAP" >&2; exit 1; }
exec ros2 launch mdetect_robot desktop_navigation.launch.py map:="$MAP"

#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
"$SCRIPT_DIR/verify_workstation.sh" --network
source_ros
log "Starting workstation SLAM Toolbox + Nav2 + RViz"
exec ros2 launch mdetect_navigation slam.launch.py

#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
MAP="${1:-}"
[[ -n "$MAP" ]] || die "Usage: $0 /absolute/path/to/map.yaml"
[[ "$MAP" = /* && -f "$MAP" ]] || die "Map must be an existing absolute YAML path: $MAP"
"$SCRIPT_DIR/verify_workstation.sh" --network
source_ros
log "Starting workstation localization + Nav2 + RViz with $MAP"
exec ros2 launch mdetect_navigation navigation.launch.py map:="$MAP"

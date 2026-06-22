#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "$SCRIPT_DIR/common.sh"
source_ros
cat <<'WARNING'
SAFETY CHECK:
  1. Lift the robot so all four wheels are completely off the ground.
  2. Keep hands, cables and clothing away from the wheels.
  3. Keep Pi bringup running and make sure the LiDAR is connected.
The test runs forward, stops, runs reverse, then verifies all four encoder speeds.
WARNING
read -r -p 'Press Enter to run the actuator test or Ctrl+C to cancel. '
exec ros2 run mdetect_base actuator_self_test

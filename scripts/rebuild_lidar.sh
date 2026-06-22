#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-$HOME/mdetect_ws}"

# Remove stale overlays before deleting a package-specific install prefix.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
rm -rf build/cspc_lidar install/cspc_lidar log/latest_build/cspc_lidar 2>/dev/null || true
colcon build --symlink-install --packages-select cspc_lidar --event-handlers console_direct+

set +u
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
ros2 pkg executables cspc_lidar

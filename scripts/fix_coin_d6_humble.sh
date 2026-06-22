#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="${1:-$HOME/mDetectRobot}"
WORKSPACE="${2:-$HOME/mdetect_ws}"
PROJECT_SOURCE="$PROJECT_ROOT/ros2_ws/src/cspc_lidar/src/node_lidar_ros.cpp"
WORKSPACE_SOURCE="$WORKSPACE/src/cspc_lidar/src/node_lidar_ros.cpp"

patch_source() {
  local source_file="$1"
  [ -f "$source_file" ] || return 0

  python3 - "$source_file" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()

old = '''\tnode->declare_parameter("port");
  \tnode->get_parameter("port", node_lidar.lidar_general_info.port);
\t
\tnode->declare_parameter("baudrate");
  \tnode->get_parameter("baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

\tnode->declare_parameter("frame_id");
  \tnode->get_parameter("frame_id", node_lidar.lidar_general_info.frame_id);
\t
\tnode->declare_parameter("version");
  \tnode->get_parameter("version", node_lidar.lidar_general_info.version);
'''

new = '''\t// ROS 2 Humble requires each statically typed parameter to be declared
\t// with either an explicit type or a default value. Launch/YAML overrides
\t// replace these defaults when the driver starts.
\tnode_lidar.lidar_general_info.port =
\t\tnode->declare_parameter<std::string>(
\t\t\t"port", node_lidar.lidar_general_info.port);

\tnode_lidar.lidar_general_info.m_SerialBaudrate =
\t\tnode->declare_parameter<int>(
\t\t\t"baudrate", node_lidar.lidar_general_info.m_SerialBaudrate);

\tnode_lidar.lidar_general_info.frame_id =
\t\tnode->declare_parameter<std::string>(
\t\t\t"frame_id", node_lidar.lidar_general_info.frame_id);

\tnode_lidar.lidar_general_info.version =
\t\tnode->declare_parameter<int>(
\t\t\t"version", node_lidar.lidar_general_info.version);
'''

if old in text:
    text = text.replace(old, new, 1)
elif 'declare_parameter<std::string>(\n\t\t\t"port"' not in text:
    raise SystemExit(f"Could not locate the old or corrected parameter block in {path}")

text = text.replace(
    'for(int i = 0;i<scan.points.size();i++)',
    'for (std::size_t i = 0; i < scan.points.size(); ++i)')
text = text.replace(
    'for(int i=0; i < scan.points.size(); i++)',
    'for (std::size_t i = 0; i < scan.points.size(); ++i)')
path.write_text(text)
print(f"Patched: {path}")
PY
}

patch_source "$PROJECT_SOURCE"
patch_source "$WORKSPACE_SOURCE"

if [ -f "$PROJECT_ROOT/scripts/install_pi.sh" ]; then
  sed -i 's#rosdep install --from-paths src --ignore-src -r -y$#rosdep install --from-paths src --ignore-src -r -y --skip-keys="ament_python"#' \
    "$PROJECT_ROOT/scripts/install_pi.sh"
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble setup file was not found."
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

cd "$WORKSPACE"
rm -rf build/cspc_lidar install/cspc_lidar
colcon build --symlink-install

echo
echo "COIN-D6 Humble correction and workspace build completed."
echo "Run: source $WORKSPACE/install/setup.bash"
echo "Then: ros2 pkg executables cspc_lidar"

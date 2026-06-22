#!/usr/bin/env bash
set -u

echo "=== Serial devices ==="
ls -l /dev/sc_mini /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
if [ -e /dev/sc_mini ]; then
  echo "COIN-D6 resolves to: $(readlink -f /dev/sc_mini)"
fi

echo
echo "=== ROS environment ==="
echo "ROS_DISTRO=${ROS_DISTRO:-not sourced}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not set}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-not set}"

echo
echo "=== COIN-D6 package ==="
ros2 pkg executables cspc_lidar 2>/dev/null || echo "cspc_lidar is not built or the workspace is not sourced."

echo
echo "=== Important topics ==="
ros2 topic list 2>/dev/null | grep -E '^/(scan|point_cloud|lsd_error|cmd_vel|cmd_vel_safe|wheel/odometry|odometry/filtered|imu/data|map|tf|tf_static)$' || true

echo
echo "=== Important nodes ==="
ros2 node list 2>/dev/null | grep -E '(cspc_lidar|serial_bridge|safety_supervisor|slam_toolbox|controller_server|planner_server|ekf_filter_node)' || true

echo
echo "=== One /scan sample (five-second timeout) ==="
timeout 5 ros2 topic echo /scan --once 2>/dev/null || echo "No /scan message received within five seconds."

#!/usr/bin/env bash
set -u

echo "=== Serial devices ==="
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true

echo
echo "=== ROS environment ==="
echo "ROS_DISTRO=${ROS_DISTRO:-not sourced}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not set}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-not set}"

echo
echo "=== Important topics ==="
ros2 topic list 2>/dev/null | grep -E '^/(scan|cmd_vel|cmd_vel_safe|wheel/odometry|odometry/filtered|imu/data|map|tf|tf_static)$' || true

echo
echo "=== Important nodes ==="
ros2 node list 2>/dev/null | grep -E '(serial_bridge|safety_supervisor|slam_toolbox|controller_server|planner_server|ekf_filter_node)' || true

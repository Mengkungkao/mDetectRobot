#!/usr/bin/env bash
# Source this file on both the Raspberry Pi and Ubuntu workstation.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
source /opt/ros/humble/setup.bash
if [ -f "$HOME/mdetect_ws/install/setup.bash" ]; then
  source "$HOME/mdetect_ws/install/setup.bash"
fi

#!/usr/bin/env bash
# Source this file on both the Raspberry Pi and Ubuntu workstation.

# Source a ROS/ament setup file without allowing `set -u` to break on
# optional environment variables such as AMENT_TRACE_SETUP_FILES.
safe_source_setup() {
  local setup_file="$1"
  local restore_nounset=0

  if [[ "$-" == *u* ]]; then
    restore_nounset=1
    set +u
  fi

  # shellcheck disable=SC1090
  source "$setup_file"

  if [ "$restore_nounset" -eq 1 ]; then
    set -u
  fi
}

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
safe_source_setup /opt/ros/humble/setup.bash
if [ -f "$HOME/mdetect_ws/install/setup.bash" ]; then
  safe_source_setup "$HOME/mdetect_ws/install/setup.bash"
fi

#!/usr/bin/env bash
# Source ROS safely even when the caller uses `set -u`.
_mdetect_had_nounset=0
case $- in *u*) _mdetect_had_nounset=1; set +u ;; esac
source /opt/ros/humble/setup.bash
if [[ -f "${MDETECT_WS:-$HOME/mdetect_ws}/install/setup.bash" ]]; then
  source "${MDETECT_WS:-$HOME/mdetect_ws}/install/setup.bash"
fi
if [[ $_mdetect_had_nounset -eq 1 ]]; then set -u; fi
unset _mdetect_had_nounset
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

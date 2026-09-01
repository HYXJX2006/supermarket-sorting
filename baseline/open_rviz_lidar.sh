#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-98}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export DISPLAY="${DISPLAY:-:0}"
exec rviz2 -d /workspace/baseline/rviz_lidar.rviz "$@"
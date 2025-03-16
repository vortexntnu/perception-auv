#!/bin/bash
# This script runs the command to execute Isaac ROS

# Define paths
CONFIG_FILE_SOURCE="/home/vortex/dockerws/src/perception-auv/.isaac_ros_common-config"
CONFIG_FILE_TARGET="/home/vortex/.isaac_ros_common-config"

# Remove existing file or symlink if it exists
rm -f "$CONFIG_FILE_TARGET"

# Create the symbolic link
ln -s "$CONFIG_FILE_SOURCE" "$CONFIG_FILE_TARGET"

$PATH_TO_ISAAC_ROS_COMMON/scripts/run_dev.sh -d ~/perception_ws/

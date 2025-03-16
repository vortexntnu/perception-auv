#!/bin/bash
# This script runs the command to execute Isaac ROS

# Default values for offline and extra flags for running development script
OFFLINE=0
EXTRA_FLAG=""

# Loop through all command line arguments
while [ "$#" -gt 0 ]; do
  case "$1" in
    --offline)
      # Shift to get the next argument as the value
      OFFLINE="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

# Echo status for debugging
echo "Offline mode is set to: $OFFLINE"

# Define paths
CONFIG_FILE_SOURCE="/home/vortex/perception_ws/src/perception-auv/.isaac_ros_common-config"
CONFIG_FILE_TARGET="/home/vortex/.isaac_ros_common-config"

# Remove existing file or symlink if it exists
rm -f "$CONFIG_FILE_TARGET"

# Create the symbolic link
ln -s "$CONFIG_FILE_SOURCE" "$CONFIG_FILE_TARGET"

# Set the extra flag if offline is 1
if [ "$OFFLINE" -eq 1 ]; then
    EXTRA_FLAG="-b 1"
else
    EXTRA_FLAG=""
fi

# Run the command with the extra flag if needed
$PATH_TO_ISAAC_ROS_COMMON/scripts/run_dev.sh $EXTRA_FLAG -d ~/perception_ws/

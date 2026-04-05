#!/usr/bin/env bash
# Source this script:
#   source unset_fastdds.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Please source this script instead of executing it:"
  echo "  source ${0}"
  return 1 2>/dev/null || exit 1
fi

unset FASTDDS_DEFAULT_PROFILES_FILE
unset RMW_IMPLEMENTATION
unset ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "Cleared Fast DDS/ROS environment variables from this shell."

#!/usr/bin/env bash

# Source this file, do not execute it.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[Fast DDS] Please source this script:"
  echo "  source ${0}"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILE_FILE="${SCRIPT_DIR}/../config/fastdds_multi_interface.xml"

if [[ ! -f "${PROFILE_FILE}" ]]; then
  echo "[Fast DDS] Error: profile not found at ${PROFILE_FILE}"
  return 1
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="${PROFILE_FILE}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "[Fast DDS] Profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "[Fast DDS] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[Fast DDS] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

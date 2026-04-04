#!/usr/bin/env bash

# Source this file, do not execute it.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[Fast DDS] Please source this script:"
  echo "  source ${0}"
  exit 1
fi

ROS_DOMAIN="0"
FASTDDS_IFACE="eno1"

# Optional check only: make sure eno1 exists, but do not require `ip`
if [[ ! -e "/sys/class/net/${FASTDDS_IFACE}" ]]; then
  echo "[Fast DDS] Warning: interface '${FASTDDS_IFACE}' not found. Skipping setup."
  return 0
fi

unset FASTDDS_DEFAULT_PROFILES_FILE
export SKIP_DEFAULT_XML=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN}"
unset ROS_LOCALHOST_ONLY

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "[Fast DDS] Enabled UDPv4-only Fast DDS, ROS_DOMAIN_ID=${ROS_DOMAIN}"
echo "[Fast DDS] FASTDDS_DEFAULT_PROFILES_FILE unset"
echo "[Fast DDS] SKIP_DEFAULT_XML=1"
echo "[Fast DDS] FASTDDS_BUILTIN_TRANSPORTS=UDPv4"
echo "[Fast DDS] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

#!/bin/bash
set -e

echo "[INFO] Starting ZED SDK install script..."

# Install wget if not present
if ! command -v wget &> /dev/null; then
    echo "[INFO] Installing wget..."
    apt-get update && apt-get install -y wget
fi

# Set paths
DOWNLOAD_DIR=/tmp
INSTALLER_NAME=ZED_SDK_Linux.run
INSTALLER_PATH="$DOWNLOAD_DIR/$INSTALLER_NAME"

echo "[INFO] Downloading ZED SDK installer to $INSTALLER_PATH..."
wget -q --no-check-certificate -O "$INSTALLER_PATH" \
  https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.2.zstd.run

chmod +x "$INSTALLER_PATH"
"$INSTALLER_PATH" silent skip_od_module skip_python skip_drivers

# Only create Jetson-specific symlink if on ARM
if [[ "$(uname -m)" == "aarch64" ]]; then
  echo "[INFO] Creating symlink for libv4l2..."
  ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so
fi

rm -f "$INSTALLER_PATH"

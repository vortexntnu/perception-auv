#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# ---------------------------------------
# Install wget if not already installed
# ---------------------------------------
# wget is needed to download the ZED SDK installer.
# Remove 'sudo' if you're running in a root environment (e.g. Docker).
if ! command -v wget &> /dev/null; then
    echo "[INFO] Installing wget..."
    sudo apt-get update && sudo apt-get install -y wget
else
    echo "[INFO] wget already installed"
fi

# ---------------------------------------
# Download the ZED SDK installer
# ---------------------------------------
echo "[INFO] Downloading ZED SDK installer..."
wget -q --no-check-certificate -O ZED_SDK_Linux.run \
    https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.2.zstd.run

# ---------------------------------------
# Make the installer executable
# ---------------------------------------
chmod +x ZED_SDK_Linux.run

# ---------------------------------------
# Run the installer in silent mode
# - skip_od_module: Skips object detection module
# - skip_python: Skips Python bindings
# - skip_drivers: Skips USB drivers (not needed on Jetson)
# ---------------------------------------
echo "[INFO] Running ZED SDK installer..."
./ZED_SDK_Linux.run silent skip_od_module skip_python skip_drivers

# ---------------------------------------
# Symlink libv4l2 to correct location for compatibility
# ---------------------------------------
echo "[INFO] Creating symlink for libv4l2..."
ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 \
       /usr/lib/aarch64-linux-gnu/libv4l2.so

# ---------------------------------------
# Clean up downloaded files and unnecessary SDK resources
# ---------------------------------------
echo "[INFO] Cleaning up..."
rm -rf /usr/local/zed/resources/*
rm -f ZED_SDK_Linux.run

echo "[INFO] ZED SDK installation completed successfully!"

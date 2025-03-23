# Set a writable download location
DOWNLOAD_DIR=/tmp
INSTALLER_NAME=ZED_SDK_Linux.run
INSTALLER_PATH="$DOWNLOAD_DIR/$INSTALLER_NAME"

# Download the installer to /tmp
echo "[INFO] Downloading ZED SDK installer to $INSTALLER_PATH..."
wget -q --no-check-certificate -O "$INSTALLER_PATH" \
  https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.2/ZED_SDK_Tegra_L4T36.4_v4.2.2.zstd.run

# Make it executable and run it
chmod +x "$INSTALLER_PATH"
"$INSTALLER_PATH" silent skip_od_module skip_python skip_drivers

# Symlink libv4l2 as before
ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# Optional cleanup (may not be needed since /tmp is cleared between jobs)
rm -f "$INSTALLER_PATH"

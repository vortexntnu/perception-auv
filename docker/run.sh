#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for the image name.
# Defaulted to latest image.
# ------------------------------------------------------------------------------
IMAGE="perception-image:latest"

# For Jetson (ARM64 / aarch64) add support for others later if needed for CI/CD
export PLATFORM="linux/arm64"

# ------------------------------------------------------------------------------
# Locate this script and the project root (mounted as a volume).
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")"

echo "======================================================================"
echo " Running container from image '$IMAGE'"
echo "   * PLATFORM (host architecture): $PLATFORM"
echo "   * Volume mount:                 $WORKSPACE -> /ros_ws"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Run the Docker container
# ------------------------------------------------------------------------------
docker run -it --rm \
    --user $(id -u):$(id -g) \
    --privileged \
    --network host \
    --ipc=host \
    -v /etc/localtime:/etc/localtime:ro \
    -v /usr/local/zed/resources:/usr/local/zed/resources \
    -v /usr/local/zed/settings:/usr/local/zed/settings \
    -v /dev:/dev \
    -v "$WORKSPACE":/ros_ws \
    --runtime nvidia \
    -w /ros_ws \
    "$IMAGE" \
    /bin/bash
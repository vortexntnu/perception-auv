#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for image name, base image, and platform.
# ------------------------------------------------------------------------------
export IMAGE="perception-image:latest"
export BASE_IMAGE="nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_adc428c7077de4984a00b63c55903b0a"

# For Jetson (ARM64 / aarch64) add support for others later if needed for CI/CD
export PLATFORM="linux/arm64"


# ------------------------------------------------------------------------------
# Locate this script and the project root (Docker build context).
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../../..")"


echo "======================================================================"
echo " Building Docker image"
echo "   * PLATFORM:       $PLATFORM"
echo "   * BASE_IMAGE:     $BASE_IMAGE"
echo "   * IMAGE:          $IMAGE"
echo "   * BUILD CONTEXT:  $WORKSPACE"
echo "======================================================================"
echo ""

# ------------------------------------------------------------------------------
# Build the Docker image using Docker Buildx
# ------------------------------------------------------------------------------
docker buildx build \
    --platform "$PLATFORM" \
    --build-arg BASE_IMAGE="$BASE_IMAGE" \
    --tag "$IMAGE" \
    --file "$SCRIPT_DIR/Dockerfile" \
    --load \
    "$WORKSPACE"

echo ""
echo "======================================================================"
echo " Successfully built image '$IMAGE' for platform '$PLATFORM'"
echo "======================================================================"
echo ""

#!/bin/bash
# Launch perception stack in a tmux session.
#
# Usage:
#   ./tmux_perception.sh                          # camera + OBB (defaults)
#   ./tmux_perception.sh realsense_d555 obb       # explicit camera + pipeline
#   ./tmux_perception.sh blackfly_s seg            # blackfly + segmentation
#
# Each component runs in its own tmux window so you can monitor/restart
# them independently.

set -euo pipefail

SESSION="perception"
CAMERA="${1:-realsense_d555}"
PIPELINE="${2:-obb}"

# Map camera name to launch file
declare -A CAMERA_LAUNCH=(
    [realsense_d555]="realsense_d555.launch.py"
    [blackfly_s]="blackfly_s.launch.py"
)

# Map pipeline name to YOLO launch file
declare -A PIPELINE_LAUNCH=(
    [detect]="yolo_detect.launch.py"
    [obb]="yolo_obb.launch.py"
    [seg]="yolo_seg.launch.py"
    [cls]="yolo_cls.launch.py"
)

CAMERA_FILE="${CAMERA_LAUNCH[$CAMERA]:-}"
PIPELINE_FILE="${PIPELINE_LAUNCH[$PIPELINE]:-}"

if [[ -z "$CAMERA_FILE" ]]; then
    echo "Unknown camera: $CAMERA (available: ${!CAMERA_LAUNCH[*]})"
    exit 1
fi

if [[ -z "$PIPELINE_FILE" ]]; then
    echo "Unknown pipeline: $PIPELINE (available: ${!PIPELINE_LAUNCH[*]})"
    exit 1
fi

# Kill existing session if present
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Window 1: Camera
tmux new-session -d -s "$SESSION" -n "camera"
tmux send-keys -t "$SESSION:camera" \
    "ros2 launch perception_setup $CAMERA_FILE" Enter

# Window 2: YOLO pipeline
tmux new-window -t "$SESSION" -n "$PIPELINE"
tmux send-keys -t "$SESSION:$PIPELINE" \
    "ros2 launch perception_setup $PIPELINE_FILE camera:=$CAMERA" Enter

# Attach
tmux attach -t "$SESSION"

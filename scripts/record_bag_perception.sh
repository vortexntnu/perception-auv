#!/bin/bash

# HOW TO USE:
# ./record_bag_perception.sh your_chosen_bag_name

if [ -z "$1" ]; then
	BAG_NAME=$(date +'%Y'%m'%d_%H-%M-%S')
else
	BAG_NAME="$1"
fi

TOPICS=(
	"/gripper_camera/image_raw"
	"/front_camera/image_raw"
)

ros2 bag record -o "$BAG_NAME" "${TOPICS[@]}"

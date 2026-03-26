"""Launch file that publishes camera info without starting the RealSense camera."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")

    camera_info_file = os.path.join(
        pkg_dir,
        "config",
        "cameras",
        "color_realsense_d555_calib.yaml",
    )

    with open(camera_info_file) as f:
        cfg = yaml.safe_load(f)

    camera_info_topic = cfg["camera_info_topic"]

    camera_info_pub = Node(
        package="perception_setup",
        executable="camera_info_publisher.py",
        name="camera_info_publisher",
        parameters=[
            {
                "camera_info_file": camera_info_file,
                "camera_info_topic": camera_info_topic,
            }
        ],
        output="screen",
    )

    return LaunchDescription([camera_info_pub])

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory("perception_setup")

    config_file = os.path.join(
        pkg_dir,
        "config",
        "d555_color_camera_factory.yaml"
    )

    with open(config_file, "r") as f:
        cfg = yaml.safe_load(f)

    camera_info_topic = cfg["camera_info_topic"]

    return LaunchDescription([

        Node(
            package="perception_setup",
            executable="camera_info_publisher.py",
            name="camera_info_publisher",
            parameters=[{
                "camera_info_file": config_file,
                "camera_info_topic": camera_info_topic,
            }],
            output="screen"
        )
    ])
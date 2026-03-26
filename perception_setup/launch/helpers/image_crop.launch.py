import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")
    config_file = os.path.join(pkg_dir, "config", "image_processing", "image_crop.yaml")

    with open(config_file) as f:
        cfg = yaml.safe_load(f)

    params = {
        "image_topic": cfg["image_topic"],
        "camera_info_topic": cfg["camera_info_topic"],
        "output_image_topic": cfg["output_image_topic"],
        "output_camera_info_topic": cfg["output_camera_info_topic"],
        "crop.x_offset": int(cfg["crop"]["x_offset"]),
        "crop.y_offset": int(cfg["crop"]["y_offset"]),
        "crop.width": int(cfg["crop"]["width"]),
        "crop.height": int(cfg["crop"]["height"]),
    }

    return LaunchDescription(
        [
            Node(
                package="perception_setup",
                executable="image_crop.py",
                name="image_crop",
                parameters=[params],
                output="screen",
            )
        ]
    )

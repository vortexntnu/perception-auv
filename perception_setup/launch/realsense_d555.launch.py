"""Wrapper launch file for the RealSense D555 camera.

See: https://github.com/vortexntnu/realsense-ros/blob/r/4.57.6/realsense2_camera/launch/rs_launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")

    default_rs_config = os.path.join(pkg_dir, "config", "realsense_d555.yaml")
    camera_info_file = os.path.join(
        pkg_dir,
        "config",
        "color_realsense_d555_calib.yaml",
    )

    with open(camera_info_file, "r") as f:
        cfg = yaml.safe_load(f)

    camera_info_topic = cfg["camera_info_topic"]

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_rs_config,
        description="Path to a YAML file with node-level parameter overrides for rs_launch.py",
    )

    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("realsense2_camera"), "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
        }.items(),
    )

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

    return LaunchDescription([
        config_file_arg,
        rs_launch,
        camera_info_pub,
    ])

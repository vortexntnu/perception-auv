"""Wrapper launch file for the RealSense D555 camera.

See: https://github.com/vortexntnu/realsense-ros/blob/r/4.57.6/realsense2_camera/launch/rs_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")
    default_config = os.path.join(pkg_dir, "config", "realsense_d555.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
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

    return LaunchDescription([config_file_arg, rs_launch])

"""Wrapper launch file for the RealSense D555 camera.

Starts:
  - RealSense driver (rs_launch.py)
  - camera_info_publisher  (publishes calibration on color_raw/camera_info)
  - image_undistort        (undistorts color image)
  - image_crop             (crops depth image)

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

    default_rs_config = os.path.join(
        pkg_dir, "config", "cameras", "realsense_d555.yaml"
    )
    camera_info_file = os.path.join(
        pkg_dir, "config", "cameras", "color_realsense_d555_calib.yaml"
    )

    cameras_path = os.path.join(pkg_dir, "config", "cameras", "cameras.yaml")
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras["realsense_d555"]

    helpers_dir = os.path.join(pkg_dir, "launch", "image_processing")

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
                "camera_info_topic": cam["calibration_camera_info_topic"],
            }
        ],
        output="screen",
    )

    image_undistort = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(helpers_dir, "image_undistort.launch.py")
        ),
    )

    image_crop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(helpers_dir, "image_crop.launch.py")
        ),
    )

    return LaunchDescription(
        [
            config_file_arg,
            rs_launch,
            camera_info_pub,
            image_undistort,
            image_crop,
        ]
    )

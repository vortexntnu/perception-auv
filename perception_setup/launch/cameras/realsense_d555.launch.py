"""RealSense D555 camera launch file.

Starts:
  - RealSense camera driver node
  - camera_info_publisher (publishes calibration CameraInfo)
  - image_undistort (undistorts color image)
  - image_crop (crops depth image)

Topics are read from cameras.yaml (single source of truth).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")

    cameras_path = os.path.join(pkg_dir, "config", "cameras", "cameras.yaml")
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras["realsense_d555"]

    calib_file = os.path.join(
        pkg_dir, "config", "cameras", "color_realsense_d555_calib.yaml"
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="front_camera",
        namespace="camera",
        parameters=[
            {
                "enable_color": True,
                "rgb_camera.color_profile": "896,504,15",
                "rgb_camera.color_format": "RGB8",
                "rgb_camera.enable_auto_exposure": True,
                "enable_depth": True,
                "depth_module.depth_profile": "896,504,15",
                "depth_module.depth_format": "Z16",
                "depth_module.enable_auto_exposure": True,
                "depth_module.emitter_enabled": False,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "enable_motion": False,
                "publish_tf": False,
                "enable_sync": False,
            }
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
    )

    camera_info_publisher_node = Node(
        package="perception_setup",
        executable="camera_info_publisher.py",
        name="camera_info_publisher",
        parameters=[
            {
                "camera_info_file": calib_file,
                "camera_info_topic": cam["calibration_camera_info_topic"],
            }
        ],
        output="screen",
    )

    image_undistort_node = Node(
        package="perception_setup",
        executable="image_undistort.py",
        name="image_undistort",
        parameters=[
            {
                "image_topic": cam["raw_image_topic"],
                "camera_info_topic": cam["calibration_camera_info_topic"],
                "raw_camera_info_topic": cam["raw_camera_info_topic"],
                "output_image_topic": cam["image_topic"],
                "output_camera_info_topic": cam["camera_info_topic"],
                "enable_undistort": True,
            }
        ],
        output="screen",
    )

    image_crop_node = Node(
        package="perception_setup",
        executable="image_crop.py",
        name="image_crop",
        parameters=[
            {
                "image_topic": cam["raw_depth_topic"],
                "camera_info_topic": cam["raw_depth_camera_info_topic"],
                "output_image_topic": cam["depth_image_topic"],
                "output_camera_info_topic": cam["depth_camera_info_topic"],
                "enable_crop": True,
                "crop.x_offset": 260,
                "crop.y_offset": 190,
                "crop.width": 485,
                "crop.height": 245,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            realsense_node,
            camera_info_publisher_node,
            image_undistort_node,
            image_crop_node,
        ]
    )

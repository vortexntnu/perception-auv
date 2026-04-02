"""RealSense D555 camera launch file.

Starts:
  - RealSense camera driver node
  - image_undistort (undistorts color image using color_realsense_d555_calib.yaml)

Topics are read from cameras.yaml (single source of truth).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

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
        name="camera",
        namespace="camera",
        parameters=[
            {
                "enable_color": True,
                "rgb_camera.color_profile": "896,504,15", # When updating the image size, make sure to also update the calibration file
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
        remappings=[
            # Color image is NOT remapped here — image_undistort reads the raw
            # topic and publishes to cam["image_topic"] (image_rect).
            (cam["raw_depth_image_topic"], cam["depth_image_topic"]),
            (cam["raw_depth_camera_info_topic"], cam["depth_camera_info_topic"]),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        emulate_tty=True,
    )

    # Undistorts the color image using the calibration YAML.
    # In undistort mode  : loads K+D from calib file at startup, publishes
    #                      rectified image + zero-distortion camera_info.
    # In passthrough mode: relays raw image and driver camera_info as-is.
    image_undistort_container = ComposableNodeContainer(
        name="image_undistort_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="perception_setup",
                plugin="perception_setup::ImageUndistort",
                name="color_image_undistort",
                parameters=[
                    {
                        "image_topic": cam["raw_color_image_topic"],
                        "camera_info_file": calib_file,
                        "raw_camera_info_topic": cam["raw_color_camera_info_topic"],
                        "output_image_topic": cam["image_topic"],
                        "output_camera_info_topic": cam["camera_info_topic"],
                        "enable_undistort": LaunchConfiguration("enable_undistort"),
                        "image_qos": "reliable",
                    }
                ],
            ),
        ],
        output="screen",
    )

    # TODO: Valve detection worked better without this, figure out why.
    # image_crop_node = Node(
    #     package="perception_setup",
    #     executable="image_crop.py",
    #     name="image_crop",
    #     parameters=[
    #         {
    #             "image_topic": cam["raw_depth_image_topic"],
    #             "camera_info_topic": cam["raw_depth_camera_info_topic"],
    #             "output_image_topic": cam["depth_image_topic"],
    #             "output_camera_info_topic": cam["depth_camera_info_topic"],
    #             "enable_crop": False,
    #             "crop.x_offset": 260,
    #             "crop.y_offset": 190,
    #             "crop.width": 485,
    #             "crop.height": 245,
    #         }
    #     ],
    #     output="screen",
    # )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_undistort",
                default_value="true",
                description="Undistort color image before publishing to image_topic",
            ),
            realsense_node,
            image_undistort_container,
        ]
    )

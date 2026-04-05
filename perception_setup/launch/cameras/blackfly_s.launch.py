"""FLIR Blackfly S downwards camera launch file.

Starts:
  - Spinnaker camera driver component in a dedicated container
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory("perception_setup")

    driver_params = os.path.join(
        pkg_dir, "config", "cameras", "blackfly_s_driver_params.yaml"
    )
    spinnaker_map = os.path.join(pkg_dir, "config", "cameras", "blackfly_s_params.yaml")
    calib_path = os.path.join(pkg_dir, "config", "cameras", "blackfly_s_calib.yaml")

    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable FLIR Blackfly S camera component",
    )

    flir_container = ComposableNodeContainer(
        name="blackfly_s_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name="blackfly_s",
                parameters=[
                    driver_params,
                    {
                        "parameter_file": spinnaker_map,
                        "serial_number": '23494258',
                        "camerainfo_url": f"file://{calib_path}",
                    },
                ],
                remappings=[("~/control", "/exposure_control/control")],
                extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(LaunchConfiguration("enable_camera")),
            )
        ],
        output="screen",
    )

    image_to_gstreamer_node = Node(
        package="image_to_gstreamer",
        executable="image_to_gstreamer_node",
        name="image_to_gstreamer_node",
        additional_env={"EGL_PLATFORM": "surfaceless"},
        parameters=[
            {
                "input_topic": LaunchConfiguration("gst_input_topic"),
                "host": LaunchConfiguration("gst_host"),
                "port": LaunchConfiguration("gst_port"),
                "bitrate": LaunchConfiguration("gst_bitrate"),
                "framerate": LaunchConfiguration("gst_framerate"),
                "preset_level": LaunchConfiguration("gst_preset_level"),
                "iframe_interval": LaunchConfiguration("gst_iframe_interval"),
                "control_rate": LaunchConfiguration("gst_control_rate"),
                "pt": LaunchConfiguration("gst_pt"),
                "config_interval": LaunchConfiguration("gst_config_interval"),
                "format": "RGB",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            enable_camera_arg,
            DeclareLaunchArgument(
                "gst_input_topic",
                default_value="/blackfly_s/image_raw",
                description="Image topic to stream via GStreamer",
            ),
            DeclareLaunchArgument(
                "gst_host",
                default_value="10.0.0.68",
                description="GStreamer stream destination host",
            ),
            DeclareLaunchArgument(
                "gst_port",
                default_value="5001",
                description="GStreamer stream destination port",
            ),
            DeclareLaunchArgument(
                "gst_bitrate",
                default_value="500000",
                description="GStreamer encoder bitrate (bps)",
            ),
            DeclareLaunchArgument(
                "gst_framerate",
                default_value="15",
                description="GStreamer stream framerate",
            ),
            DeclareLaunchArgument(
                "gst_preset_level",
                default_value="1",
                description="GStreamer encoder preset level",
            ),
            DeclareLaunchArgument(
                "gst_iframe_interval",
                default_value="15",
                description="GStreamer I-frame interval",
            ),
            DeclareLaunchArgument(
                "gst_control_rate",
                default_value="1",
                description="GStreamer control rate",
            ),
            DeclareLaunchArgument(
                "gst_pt",
                default_value="96",
                description="GStreamer RTP payload type",
            ),
            DeclareLaunchArgument(
                "gst_config_interval",
                default_value="1",
                description="GStreamer config interval",
            ),
            flir_container,
            image_to_gstreamer_node,
        ]
    )

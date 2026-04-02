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
from launch_ros.actions import ComposableNodeContainer
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

    return LaunchDescription(
        [
            enable_camera_arg,
            flir_container,
        ]
    )

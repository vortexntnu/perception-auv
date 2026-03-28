"""Wrapper launch file for the FLIR Blackfly S BFS-PGE-16S2C-CS downwards camera.

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

    default_cam_config = os.path.join(
        pkg_dir, "config", "cameras", "bfs_pge_16s2c_params.yaml"
    )
    default_spinnaker_map = os.path.join(
        pkg_dir, "config", "cameras", "blackfly_s_params.yaml"
    )
    calib_path = os.path.join(pkg_dir, "config", "cameras", "bfs_pge_16s2c_calib.yaml")
    calib_url = f"file://{calib_path}"

    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="true",
        description="Enable FLIR BFS-PGE-16S2C-CS camera component",
    )

    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="bfs_pge_16s2c",
        description="Camera node name (also used as topic namespace by the driver)",
    )
    serial_arg = DeclareLaunchArgument(
        "serial",
        default_value="",
        description="FLIR serial number (empty selects first available camera)",
    )
    camera_params_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=default_cam_config,
        description="Path to ROS parameters for camera_driver_node",
    )
    parameter_file_arg = DeclareLaunchArgument(
        "parameter_file",
        default_value=default_spinnaker_map,
        description="Path to Spinnaker node-map parameter definition YAML",
    )

    flir_container = ComposableNodeContainer(
        name="bfs_pge_16s2c_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name=LaunchConfiguration("camera_name"),
                parameters=[
                    LaunchConfiguration("camera_params_file"),
                    {
                        "parameter_file": LaunchConfiguration("parameter_file"),
                        "serial_number": LaunchConfiguration("serial"),
                        "camerainfo_url": calib_url,
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
            camera_name_arg,
            serial_arg,
            camera_params_arg,
            parameter_file_arg,
            flir_container,
        ]
    )

"""FLIR Blackfly S downwards camera launch file.

Starts:
  - Spinnaker camera driver component in a dedicated container
  - image_to_gstreamer to stream the raw image over RTP/H.265
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
    pkg_dir = get_package_share_directory('perception_setup')

    driver_params = os.path.join(
        pkg_dir, 'config', 'cameras', 'blackfly_s_driver_params.yaml'
    )
    spinnaker_map = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_params.yaml')
    calib_path = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_calib.yaml')

    flir_container = ComposableNodeContainer(
        name='blackfly_s_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='blackfly_s',
                parameters=[
                    driver_params,
                    {
                        'parameter_file': spinnaker_map,
                        'serial_number': '23494258',
                        'camerainfo_url': f'file://{calib_path}',
                    },
                ],
                remappings=[('~/control', '/exposure_control/control')],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(LaunchConfiguration('enable_camera')),
            )
        ],
        output='screen',
    )

    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': '/blackfly_s/image_raw',
                'host': '10.0.0.68',
                'port': 5001,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'RGB',
            }
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description='Enable FLIR Blackfly S camera component',
            ),
            flir_container,
            image_to_gstreamer_node,
        ]
    )

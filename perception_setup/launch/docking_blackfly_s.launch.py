"""Blackfly S -> ArUco Detection -> GStreamer stream.

Pipeline:
1. Blackfly S camera driver publishes raw BGR image
2. aruco_detector runs on the raw image, publishes detections and visualization
3. image_to_gstreamer streams the ArUco visualization image over RTP/H.265
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

    drone = LaunchConfiguration('drone')

    docking_container = ComposableNodeContainer(
        name='docking_blackfly_s_container',
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
                remappings=[
                    ('~/control', '/exposure_control/control'),
                    ('/blackfly_s/image_raw',   ['/', drone, '/down_camera/image_color']),
                    ('/blackfly_s/camera_info', ['/', drone, '/down_camera/camera_info']),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(LaunchConfiguration('enable_camera')),
            ),
            ComposableNode(
                package='aruco_detector',
                plugin='ArucoDetectorNode',
                name='aruco_detector_node',
                parameters=[
                    os.path.join(
                        get_package_share_directory('aruco_detector'),
                        'config',
                        'aruco_detector_params.yaml',
                    ),
                    {
                        'subs.image_topic': ['/', drone, '/down_camera/image_color'],
                        'subs.camera_info_topic': ['/', drone, '/down_camera/camera_info'],
                        'pubs.aruco_image': '/down_cam/aruco_detector/image',
                        'out_tf_frame': 'nautilus/downwards_camera_optical',
                    },
                ],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': '/down_cam/aruco_detector/image',
                'host': '10.0.0.169',
                'port': 5001,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'BGR',
            }
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Drone name, prepended to all published topics (e.g. /nautilus/down_camera/image_color)',
            ),
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description='Enable FLIR Blackfly S camera component',
            ),
            docking_container,
            image_to_gstreamer_node,
        ]
    )

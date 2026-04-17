"""RealSense D555 -> ArUco Detection -> GStreamer stream.

Pipeline:
1. RealSense D555 camera driver publishes raw RGB color image
2. image_undistort undistorts the raw color image
3. aruco_detector runs on the undistorted image, publishes detections and visualization
4. image_to_gstreamer streams the ArUco visualization image over RTP/H.265
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )

    docking_container = ComposableNodeContainer(
        name='docking_realsense_d555_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                namespace='camera',
                parameters=[
                    {
                        'enable_color': True,
                        'rgb_camera.color_profile': '896,504,15',
                        'rgb_camera.color_format': 'RGB8',
                        'rgb_camera.enable_auto_exposure': True,
                        'enable_depth': False,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_gyro': False,
                        'enable_accel': False,
                        'enable_motion': False,
                        'publish_tf': False,
                        'enable_sync': False,
                    }
                ],
            ),
            ComposableNode(
                package='perception_setup',
                plugin='perception_setup::ImageUndistort',
                name='color_image_undistort',
                parameters=[
                    {
                        'image_topic': '/camera/camera/color/image_raw',
                        'camera_info_file': calib_file,
                        'raw_camera_info_topic': '/camera/camera/color/camera_info',
                        'output_image_topic': '/realsense_d555/color/image_rect',
                        'output_camera_info_topic': '/realsense_d555/color/camera_info',
                        'enable_undistort': LaunchConfiguration('enable_undistort'),
                        'image_qos': 'sensor_data',
                    }
                ],
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
                        'subs.image_topic': '/realsense_d555/color/image_rect',
                        'subs.camera_info_topic': '/realsense_d555/color/camera_info',
                        'pubs.aruco_image': '/forward_cam/aruco_detector/image',
                        'out_tf_frame': 'nautilus/front_camera_optical',
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
                'input_topic': '/forward_cam/aruco_detector/image',
                'host': '10.0.0.169',
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
                'enable_undistort',
                default_value='true',
                description='Undistort color image before ArUco detection',
            ),
            docking_container,
            image_to_gstreamer_node,
        ]
    )

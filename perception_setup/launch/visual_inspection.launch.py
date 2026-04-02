"""RealSense D555 -> Image Filtering -> ArUco Detection.

Pipeline:
1. RealSense D555 camera publishes raw color image
2. image_undistort undistorts the raw color image
3. image_filtering to filter the undistorted image
4. aruco_detector runs on the filtered image, publishes detections, visualization and writes logs
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

FILTERED_IMAGE_TOPIC = '/visual_inspection/filtered_image'


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')

    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras['realsense_d555']

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )

    visual_inspection_container = ComposableNodeContainer(
        name='visual_inspection_container',
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
                        'image_topic': cam['raw_color_image_topic'],
                        'camera_info_file': calib_file,
                        'raw_camera_info_topic': cam['raw_color_camera_info_topic'],
                        'output_image_topic': cam['image_topic'],
                        'output_camera_info_topic': cam['camera_info_topic'],
                        'enable_undistort': LaunchConfiguration('enable_undistort'),
                        'image_qos': 'sensor_data',
                    }
                ],
            ),
            ComposableNode(
                package='image_filtering',
                plugin='ImageFilteringNode',
                name='image_filtering_node',
                parameters=[
                    os.path.join(
                        get_package_share_directory('image_filtering'),
                        'config',
                        'image_filtering_params.yaml',
                    ),
                    {
                        'sub_topic': cam['image_topic'],
                        'pub_topic': FILTERED_IMAGE_TOPIC,
                        'input_encoding': cam['encoding'],
                        'output_encoding': cam['encoding'],
                        'filter_params.filter_type': 'no_filter',
                    },
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
                        'subs.image_topic': FILTERED_IMAGE_TOPIC,
                        'subs.camera_info_topic': cam['camera_info_topic'],
                        'detect_board': False,
                        'visualize': True,
                        'log_markers': True,
                        'publish_detections': True,
                        'publish_landmarks': False,
                    },
                ],
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Undistort color image before publishing to image_topic',
            ),
            visual_inspection_container,
        ]
    )

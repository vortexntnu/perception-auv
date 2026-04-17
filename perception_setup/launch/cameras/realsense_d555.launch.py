"""RealSense D555 camera launch file.

Starts:
  - RealSense camera driver node (raw color + depth)
  - image_undistort (undistorts color image using color_realsense_d555_calib.yaml)
  - image_to_gstreamer to stream the undistorted color image
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

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[
            {
                'enable_color': True,
                # NB: When updating the image size, also update the calibration file
                'rgb_camera.color_profile': '896,504,15',
                'rgb_camera.color_format': 'BGR8',
                'rgb_camera.enable_auto_exposure': True,
                'enable_depth': True,
                'depth_module.depth_profile': '896,504,15',
                'depth_module.depth_format': 'Z16',
                'depth_module.enable_auto_exposure': True,
                'depth_module.emitter_enabled': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_gyro': False,
                'enable_accel': False,
                'enable_motion': False,
                'publish_tf': False,
                'enable_sync': False,
            }
        ],
        remappings=[
            ('/camera/camera/depth/image_rect_raw', '/camera/camera/depth/image_rect_raw'),
            ('/camera/camera/depth/camera_info', '/camera/camera/depth/camera_info'),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )

    image_undistort_container = ComposableNodeContainer(
        name='image_undistort_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
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
                        'image_qos': 'reliable',
                    }
                ],
            ),
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
                'input_topic': '/realsense_d555/color/image_rect',
                'host': '10.0.0.169',
                'port': 5000,
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
                description='Undistort color image before publishing to image_topic',
            ),
            realsense_node,
            image_undistort_container,
            image_to_gstreamer_node,
        ]
    )

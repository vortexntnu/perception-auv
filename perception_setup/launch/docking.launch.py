import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    flir_aruco = LaunchConfiguration('flir_aruco')
    flir_aruco_arg = DeclareLaunchArgument(
        'flir_aruco',
        default_value='true',
        description='enable flir Aruco detection',
    )

    zed_aruco = LaunchConfiguration('zed_aruco')
    zed_aruco_arg = DeclareLaunchArgument(
        'zed_aruco',
        default_value='true',
        description='enable zed Aruco detection',
    )

    flir_camera = LaunchConfiguration('flir_camera')
    flir_camera_arg = DeclareLaunchArgument(
        'flir_camera',
        default_value='true',
        description='enable flir camera',
    )

    zed_camera = LaunchConfiguration('zed_camera')
    zed_camera_arg = DeclareLaunchArgument(
        'zed_camera',
        default_value='true',
        description='enable zed camera',
    )

    aruco_params_file = os.path.join(
        get_package_share_directory('aruco_detector'),
        'config',
        'aruco_detector_params.yaml',
    )

    zed_config = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'zed_driver_params.yaml',
    )

    downwards_cam_params_file = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'downwards_cam_params.yaml',
    )

    blackfly_s_config_file = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'blackfly_s_params.yaml',
    )

    downwards_cam_calib_path = os.path.join(
        get_package_share_directory('perception_setup'), 'config', 'downwards_cam_calib.yaml'
    )

    # Add 'file://' prefix to the path required by the CameraInfoManager that sets the camera calibration in the spinnaker driver
    downwards_cam_calib_url = f'file://{downwards_cam_calib_path}'

    zed_node = ComposableNode(
        package='zed_components',
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[
            zed_config,
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition=IfCondition(zed_camera),
    )

    flir_node = ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='downwards_cam',
                parameters=[
                    downwards_cam_params_file,
                    {
                        'parameter_file': blackfly_s_config_file,
                        'serial_number': '23494258',
                        'camerainfo_url': downwards_cam_calib_url,
                    },
                ],
                remappings=[
                    ('~/control', '/exposure_control/control'),
                    ('/flir_camera/image_raw', '/downwards_cam/image_raw'),
                    ('/flir_camera/camera_info', '/downwards_cam/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(flir_camera),
            )

    flir_aruco_node = ComposableNode(
                package='aruco_detector',
                plugin='ArucoDetectorNode',
                name='flir_aruco_detector',
                parameters=[aruco_params_file],
                remappings=[
                    ('/image', '/downwards_cam/image_raw'),
                    ('/camera_info', '/downwards_cam/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(flir_aruco),
            )

    zed_aruco_node = ComposableNode(
        package='aruco_detector',
        plugin='ArucoDetectorNode',
        name='zed_aruco_detector',
        parameters=[aruco_params_file],
        remappings=[
            ('/image', 'zed_node/left/image_rect_color'),
            ('/camera_info', 'zed_node/left/camera_info'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition=IfCondition(zed_aruco),
    )

    composable_node_container = ComposableNodeContainer(
        name='docking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            zed_node,
            flir_node,
            flir_aruco_node,
            zed_aruco_node,
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            flir_aruco_arg,
            zed_aruco_arg,
            flir_camera_arg,
            zed_camera_arg,
            composable_node_container,
        ]
    )




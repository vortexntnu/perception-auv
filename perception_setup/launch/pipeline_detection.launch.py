import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_filtering = LaunchConfiguration('enable_filtering')
    enable_filtering_arg = DeclareLaunchArgument(
        'enable_filtering',
        default_value='True',
        description='enable pipeline filtering',
    )

    enable_line_fitting = LaunchConfiguration('enable_line_fitting')
    enable_line_fitting_arg = DeclareLaunchArgument(
        'enable_line_fitting',
        default_value='True',
        description='enable pipeline line fitting',
    )

    enable_downwards_camera = LaunchConfiguration('enable_downwards_camera')
    enable_downwards_camera_arg = DeclareLaunchArgument(
        'enable_downwards_camera',
        default_value='True',
        description='enable flir camera',
    )

    filtering_params_file = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'pipeline_filtering_params.yaml',
    )

    filtering_line_fitting_params_file = os.path.join(
        get_package_share_directory('perception_setup'),
        'config',
        'pipeline_line_fitting_params.yaml',
    )

    downwards_cam_params_file = PathJoinSubstitution(
        [FindPackageShare('perception_setup'), 'config', 'downwards_cam_params.yaml']
    )

    blackfly_s_config_file = PathJoinSubstitution(
        [FindPackageShare('perception_setup'), 'config', 'blackfly_s_params.yaml']
    )

    package_share_directory = get_package_share_directory('perception_setup')
    downwards_cam_calib_path = os.path.join(
        package_share_directory, 'config', 'downwards_cam_calib.yaml'
    )

    # Add 'file://' prefix to the path required by the CameraInfoManager that sets the camera calibration in the spinnaker driver
    downwards_cam_calib_url = f'file://{downwards_cam_calib_path}'

    composable_node_container = ComposableNodeContainer(
        name='pipeline_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='pipeline_filtering',
                plugin='vortex::pipeline_processing::PipelineFilteringNode',
                name='pipeline_filtering_node',
                parameters=[filtering_params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(enable_filtering),
            ),
            ComposableNode(
                package='pipeline_line_fitting',
                plugin='PipelineLineFittingNode',
                name='pipeline_line_fitting_node',
                parameters=[filtering_line_fitting_params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(enable_line_fitting),
            ),
            ComposableNode(
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
                condition=IfCondition(enable_downwards_camera),
            ),
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            enable_filtering_arg,
            enable_line_fitting_arg,
            enable_downwards_camera_arg,
            composable_node_container,
        ]
    )


generate_launch_description()

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

    enable_gripper_camera = LaunchConfiguration('enable_gripper_camera')
    enable_gripper_camera_arg = DeclareLaunchArgument(
        'enable_gripper_camera',
        default_value='False',
        description='enable gripper camera',
    )

    enable_front_camera = LaunchConfiguration('enable_front_camera')
    enable_front_camera_arg = DeclareLaunchArgument(
        'enable_front_camera',
        default_value='False',
        description='enable front camera',
    )

    enable_composable_nodes = LaunchConfiguration('enable_composable_nodes')
    enable_composable_nodes_arg = DeclareLaunchArgument(
        'enable_composable_nodes',
        default_value='True',
        description='enable composable nodes',
    )

    gripper_params_file = PathJoinSubstitution(
        [FindPackageShare('perception_setup'), 'config', 'gripper_camera_params.yaml']
    )

    front_camera_params_file = PathJoinSubstitution(
        [FindPackageShare('perception_setup'), 'config', 'front_camera_params.yaml']
    )

    blackfly_s_config_file = PathJoinSubstitution(
        [FindPackageShare('perception_setup'), 'config', 'blackfly_s_params.yaml']
    )

    package_share_directory = get_package_share_directory('perception_setup')
    gripper_camera_calib_path = os.path.join(
        package_share_directory, 'config', 'gripper_camera_calib.yaml'
    )
    # gripper_camera_calib_path = os.path.join(package_share_directory, 'config', 'gripper_camera_calib.yaml')
    front_camera_calib_path = os.path.join(
        package_share_directory, 'config', 'front_camera_calib.yaml'
    )

    # Add 'file://' prefix to the path required by the CameraInfoManager that sets the camera calibration in the spinnaker driver
    gripper_camera_calib_url = f'file://{gripper_camera_calib_path}'
    front_camera_calib_url = f'file://{front_camera_calib_path}'

    composable_node_container = ComposableNodeContainer(
        name='perception_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='gripper_camera',
                parameters=[
                    gripper_params_file,
                    {
                        'parameter_file': blackfly_s_config_file,
                        'serial_number': '23494258',
                        'camerainfo_url': gripper_camera_calib_url,
                    },
                ],
                remappings=[
                    ('~/control', '/exposure_control/control'),
                    ('/flir_camera/image_raw', '/gripper_camera/image_raw'),
                    ('/flir_camera/camera_info', '/gripper_camera/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': False}],
                condition=IfCondition(enable_gripper_camera),
            ),
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='front_camera',
                parameters=[
                    front_camera_params_file,
                    {
                        'parameter_file': blackfly_s_config_file,
                        'serial_number': '23494259',
                        'camerainfo_url': front_camera_calib_url,
                    },
                ],
                remappings=[
                    ('~/control', '/exposure_control/control'),
                    ('/flir_camera/image_raw', '/front_camera/image_raw'),
                    ('/flir_camera/camera_info', '/front_camera/camera_info'),
                ],
                extra_arguments=[{'use_intra_process_comms': False}],
                condition=IfCondition(enable_front_camera),
            ),
            ComposableNode(
                package='isaac_ros_h264_encoder',
                plugin='nvidia::isaac_ros::h264_encoder::EncoderNode',
                name='h264_encoder',
                parameters=[{
                    'input_width': 1440,
                    'input_height': 1080,
                }],
                remappings=[
                    ('image_raw', '/image_rect'),
                    ('image_compressed', '/image_compressed')]
            ),
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_left',
                parameters=[
                    {
                        'encoding_desired': 'rgb8',
                        'image_width': 1440,
                        'image_height': 1080,
                    }
                ],
                remappings=[
                    ('image_raw', '/front_camera/image_raw'),
                    ('image', 'image_rect'),
                ],
            ),
        ],
        output='screen',
        condition=IfCondition(enable_composable_nodes),
    )

    

    return LaunchDescription(
        [
            enable_gripper_camera_arg,
            enable_front_camera_arg,
            enable_composable_nodes_arg,
            composable_node_container,

        ]
    )


generate_launch_description()

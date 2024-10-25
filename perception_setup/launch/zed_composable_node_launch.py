# zed_composable_node_launch.py

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command

# Define the default container name for composable nodes
DEFAULT_CONTAINER_NAME = 'my_perception_container'

def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('container_name', DEFAULT_CONTAINER_NAME)
    args.add_arg('run_standalone', 'False')
    actions = args.get_launch_actions()

    # Configuration file path
    config_file_common = lu.get_path('perception_setup', 'config/zed_driver_params.yaml')

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = lu.get_path('zed_wrapper', 'urdf/zed_descr.urdf.xacro')

    # Robot State Publisher node (publishing static TFs for the camera)
    actions.append(
        Node(
            package='robot_state_publisher',
            namespace='zed',
            executable='robot_state_publisher',
            name='zed_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro',
                    ' ',
                    str(xacro_path),
                    ' ',
                    'camera_name:=zed',
                    ' ',
                    'camera_model:=zed2i',
                ])
            }]
        )
    )

    # Define the ZED composable node
    zed_node = ComposableNode(
        package='zed_components',
        namespace='zed',
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[
            config_file_common,  # Common parameters
            {'general.camera_name': 'zed'}
        ],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    # Add the container if 'run_standalone' is False, and load the composable node into it
    actions.append(
        lu.component_container(
            args.container_name, condition=IfCondition(lu.is_true(args.run_standalone))
        )
    )
    actions.append(
        lu.load_composable_nodes(
            args.container_name,
            [zed_node],
            condition=IfCondition(lu.is_false(args.run_standalone))
        )
    )

    return LaunchDescription(actions)

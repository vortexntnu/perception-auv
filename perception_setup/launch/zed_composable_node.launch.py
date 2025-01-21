from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
import os

# Define the default container name for composable nodes
DEFAULT_CONTAINER_NAME = 'my_perception_container'

def generate_launch_description() -> LaunchDescription:
    # Declare launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value=DEFAULT_CONTAINER_NAME,
        description='Name of the container to load the composable nodes into'
    )
    run_composable_arg = DeclareLaunchArgument(
        'run_composable',
        default_value='False',
        description='Run the ZED node as a standalone node if set to false'
    )

    container_name = LaunchConfiguration('container_name')
    run_composable = LaunchConfiguration('run_composable')

    # Configuration file path (adjust the path according to your setup)
    config_file_common = os.path.join(get_package_share_directory('perception_setup'), 'config', 'zed_driver_params.yaml')

    # URDF/xacro file to be loaded by the Robot State Publisher node
    xacro_path = os.path.join(get_package_share_directory('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro')

    # Robot State Publisher node (publishing static TFs for the camera)
    robot_state_publisher_node = Node(
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

    # Define the ZED composable node
    zed_composable_node = ComposableNode(
        package='zed_components',
        namespace='zed',
        name='zed_node',
        plugin='stereolabs::ZedCamera',
        parameters=[
            config_file_common,  # Common parameters
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
        condition=IfCondition(run_composable)
    )

    zed_node = Node(
        package='zed_wrapper',
        namespace='zed',
        executable='zed_wrapper',
        name='zed_node',
        output='screen',
        parameters=[
            config_file_common,  # Common parameters
        ],
        condition=UnlessCondition(run_composable),
    )

    # Load the ZED composable node into the existing container
    load_zed_node = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[zed_composable_node],
        condition=IfCondition(run_composable),
    )

    return LaunchDescription([
        container_name_arg,
        run_composable_arg,
        robot_state_publisher_node,
        load_zed_node,
        zed_node,
    ])
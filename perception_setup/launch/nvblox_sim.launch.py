import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition, IfCondition

# Define the default container name for composable nodes
DEFAULT_CONTAINER_NAME = 'nvblox_container'

def generate_launch_description():
    # Declare common launch arguments
    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value=DEFAULT_CONTAINER_NAME,
        description='Name of the container to load the composable nodes into'
    )
    run_standalone_arg = DeclareLaunchArgument(
        'run_standalone',
        default_value='True',
        description='Run the nodes as standalone if set to True'
    )

    # Launch configuration for the container name and run_standalone
    container_name = LaunchConfiguration('container_name')
    run_standalone = LaunchConfiguration('run_standalone')

    config_file_common = os.path.join(get_package_share_directory('perception_setup'), 'config', 'nvblox_base.yaml')

    sim_remappings = []
    sim_remappings.append(('camera_0/depth/image', '/zed/zed_node/depth/depth_registered'))
    sim_remappings.append(('camera_0/depth/camera_info', '/zed/zed_node/depth/camera_info'))
    sim_remappings.append(('camera_0/color/image', '/zed/zed_node/rgb/image_rect_color'))
    sim_remappings.append(('camera_0/color/camera_info', '/zed/zed_node/rgb/camera_info'))

    nvblox_composable_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        namespace='nvblox',
        parameters=[config_file_common]
    )

    # Define the standalone node
    nvblox_standalone_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        namespace='nvblox',
        output='screen',
        parameters=[config_file_common],
        condition=IfCondition(run_standalone),
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[nvblox_composable_node],
        condition=UnlessCondition(run_standalone),
    )
    
    

    # Combine the ZED launch and the YOLOv8 setup into a single launch description
    return LaunchDescription([
        container_name_arg,
        run_standalone_arg,
        nvblox_standalone_node,
        load_composable_nodes
       
    ]
      
    )

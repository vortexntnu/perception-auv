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
    sim_remappings.append(('camera_0/depth/image', '/depth_cam/image_depth'))
    sim_remappings.append(('camera_0/depth/camera_info', '/depth_cam/camera_info'))
    sim_remappings.append(('camera_0/color/image', '/cam/image_color'))
    sim_remappings.append(('camera_0/color/camera_info', '/cam/camera_info'))

    nvblox_composable_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        namespace='nvblox',
        parameters=[config_file_common],
        remappings=sim_remappings,
    )

    # Define the standalone node
    nvblox_standalone_node = Node(
        package='nvblox_ros',
        executable='nvblox_node',
        name='nvblox_node',
        namespace='nvblox',
        output='screen',
        parameters=[config_file_common],
        remappings=sim_remappings,
        condition=IfCondition(run_standalone),
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[nvblox_composable_node],
        condition=UnlessCondition(run_standalone),
    )

    rviz_sim_config = os.path.join(get_package_share_directory('perception_setup'), 'config', 'sim_nvblox.rviz')

    rviz_sim_node =Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(rviz_sim_config)],
            output="screen")
    
    
    

    # Combine the ZED launch and the YOLOv8 setup into a single launch description
    return LaunchDescription([
        container_name_arg,
        run_standalone_arg,
        nvblox_standalone_node,
        load_composable_nodes,
        rviz_sim_node
       
    ]
      
    )

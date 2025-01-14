import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare Launch Arguments for Octomap server
    launch_arguments = [
        DeclareLaunchArgument('input_cloud_topic', default_value='/points'),
        DeclareLaunchArgument('resolution', default_value='0.1'),
        DeclareLaunchArgument('frame_id', default_value='odom'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('height_map', default_value='True'),
        DeclareLaunchArgument('colored_map', default_value='True'),
        DeclareLaunchArgument('color_factor', default_value='0.8'),
        DeclareLaunchArgument('filter_ground', default_value='False'),
        DeclareLaunchArgument('filter_speckles', default_value='False'),
        DeclareLaunchArgument('ground_filter/distance', default_value='0.04'),
        DeclareLaunchArgument('ground_filter/angle', default_value='0.15'),
        DeclareLaunchArgument('ground_filter/plane_distance', default_value='0.07'),
        DeclareLaunchArgument('compress_map', default_value='True'),
        DeclareLaunchArgument('incremental_2D_projection', default_value='False'),
        DeclareLaunchArgument('sensor_model/max_range', default_value='-1.0'),
        DeclareLaunchArgument('sensor_model/hit', default_value='0.7'),
        DeclareLaunchArgument('sensor_model/miss', default_value='0.4'),
        DeclareLaunchArgument('sensor_model/min', default_value='0.12'),
        DeclareLaunchArgument('sensor_model/max', default_value='0.97'),
        DeclareLaunchArgument('color/r', default_value='0.0'),
        DeclareLaunchArgument('color/g', default_value='0.0'),
        DeclareLaunchArgument('color/b', default_value='1.0'),
        DeclareLaunchArgument('color/a', default_value='1.0'),
        DeclareLaunchArgument('color_free/r', default_value='0.0'),
        DeclareLaunchArgument('color_free/g', default_value='0.0'),
        DeclareLaunchArgument('color_free/b', default_value='1.0'),
        DeclareLaunchArgument('color_free/a', default_value='1.0'),
        DeclareLaunchArgument('publish_free_space', default_value='False')
    ]

    # Define the Octomap composable node
    octomap_node = ComposableNode(
        package='octomap_server2',
        plugin='octomap_server::OctomapServer',
        name='octomap_server',
        remappings=[('cloud_in', LaunchConfiguration('input_cloud_topic'))],
        parameters=[{
            'resolution': LaunchConfiguration('resolution'),
            'frame_id': LaunchConfiguration('frame_id'),
            'base_frame_id': LaunchConfiguration('base_frame_id'),
            'height_map': LaunchConfiguration('height_map'),
            'colored_map': LaunchConfiguration('colored_map'),
            'color_factor': LaunchConfiguration('color_factor'),
            'filter_ground': LaunchConfiguration('filter_ground'),
            'filter_speckles': LaunchConfiguration('filter_speckles'),
            'ground_filter/distance': LaunchConfiguration('ground_filter/distance'),
            'ground_filter/angle': LaunchConfiguration('ground_filter/angle'),
            'ground_filter/plane_distance': LaunchConfiguration('ground_filter/plane_distance'),
            'compress_map': LaunchConfiguration('compress_map'),
            'incremental_2D_projection': LaunchConfiguration('incremental_2D_projection'),
            'sensor_model/max_range': LaunchConfiguration('sensor_model/max_range'),
            'sensor_model/hit': LaunchConfiguration('sensor_model/hit'),
            'sensor_model/miss': LaunchConfiguration('sensor_model/miss'),
            'sensor_model/min': LaunchConfiguration('sensor_model/min'),
            'sensor_model/max': LaunchConfiguration('sensor_model/max'),
            'color/r': LaunchConfiguration('color/r'),
            'color/g': LaunchConfiguration('color/g'),
            'color/b': LaunchConfiguration('color/b'),
            'color/a': LaunchConfiguration('color/a'),
            'color_free/r': LaunchConfiguration('color_free/r'),
            'color_free/g': LaunchConfiguration('color_free/g'),
            'color_free/b': LaunchConfiguration('color_free/b'),
            'color_free/a': LaunchConfiguration('color_free/a'),
            'publish_free_space': LaunchConfiguration('publish_free_space')
        }]
    )

    depth_to_pcl_node = ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[('image_rect', '/depth_cam/image_depth'),
                                ('camera_info', '/depth_cam/camera_info')]
                )

    # Create the container to hold both composable nodes
    composable_node_container = ComposableNodeContainer(
        name='point_cloud_xyz_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[depth_to_pcl_node, octomap_node],
        output='screen'
    )

    return LaunchDescription(launch_arguments + [composable_node_container])

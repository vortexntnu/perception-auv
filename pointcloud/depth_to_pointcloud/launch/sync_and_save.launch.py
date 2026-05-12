"""Launch the sync_and_save node on ROS_DOMAIN_ID=42."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(
            package='depth_to_pointcloud',
            executable='sync_and_save_node',
            name='sync_and_save_node',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/camera/depth/image_rect_raw_cropped',
                'color_topic': '/camera/camera/color/image_raw',
                'output_dir': '/home/kluge7/workspaces/isaac_ros-dev/src/pointcloud/data',
                'save_every_nth': 5,
                'sync_policy': 'approximate',
                'slop_seconds': 0.05,
                'queue_size': 30,
                # Colormap scaling range in raw 16UC1 depth units (millimetres).
                # Min ~0 mm clips near-zero noise; max ~1125 mm (~1.1 m) covers the
                # expected close-range operating distance for valve interaction.
                'depth_colormap_value_min': 0.1,
                'depth_colormap_value_max': 1125.5,
            }],
        ),
    ])

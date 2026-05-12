"""Launch the depth_to_pointcloud node on ROS_DOMAIN_ID=42."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(
            package='depth_to_pointcloud',
            executable='depth_to_pointcloud_node',
            name='depth_to_pointcloud_node',
            output='screen',
            parameters=[{
                'depth_topic': '/camera/camera/depth/image_rect_raw',
                'camera_info_topic': '/camera/camera/depth/camera_info',
                'pointcloud_topic': '/camera/camera/depth/points',
                'depth_scale': 0.001,
                'min_depth': 0.1,
                'max_depth': 10.0,
                'stride': 1,
            }],
        ),
    ])

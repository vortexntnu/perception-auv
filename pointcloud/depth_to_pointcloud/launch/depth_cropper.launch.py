"""Launch the depth_cropper node on ROS_DOMAIN_ID=42."""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '42'),
        Node(
            package='depth_to_pointcloud',
            executable='depth_cropper_node',
            name='depth_cropper_node',
            output='screen',
            parameters=[{
                'depth_topic_in': '/camera/camera/depth/image_rect_raw',
                'info_topic_in': '/camera/camera/depth/camera_info',
                'depth_topic_out': '/camera/camera/depth/image_rect_raw_cropped',
                'info_topic_out': '/camera/camera/depth/camera_info_cropped',
                'x_offset': 260,
                'y_offset': 190,
                'width': 485,
                'height': 245,
                'max_depth_value': 2500.0,
            }],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/depth/image_rect_raw',
            description='RealSense depth image topic',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/depth/camera_info',
            description='RealSense depth camera_info topic',
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/depth/pointcloud',
            description='Output PointCloud2 topic',
        ),
        DeclareLaunchArgument(
            'depth_scale',
            default_value='0.001',
            description='Scale factor from raw depth units to meters (RealSense default: 0.001)',
        ),

        Node(
            package='depth_to_point',
            executable='depth_to_point_node',
            name='depth_to_pointcloud',
            output='screen',
            parameters=[{
                'depth_topic':       LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'pointcloud_topic':  LaunchConfiguration('pointcloud_topic'),
                'depth_scale':       LaunchConfiguration('depth_scale'),
            }],
        ),
    ])

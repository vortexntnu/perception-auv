import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('norbit_fls_ros_interface'),
        'config',
        'norbit_fls_ros_interface_params.yaml',
    )

    namespace = LaunchConfiguration('namespace')

    norbit_fls_ros_interface_node = Node(
        package='norbit_fls_ros_interface',
        executable='norbit_fls_ros_interface_node',
        name='norbit_fls_ros_interface_node',
        namespace=namespace,
        parameters=[config],
        output='screen',
    )

    sonar_overlay_node = Node(
        package='norbit_fls_ros_interface',
        executable='sonar_overlay_node',
        name='sonar_overlay_node',
        namespace=namespace,
        parameters=[config],
        output='screen',
    )

    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': '/fls_image/display_mono',
                'host': '10.0.0.68',
                'port': 5002,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'GRAY8',
            }
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the norbit_fls_ros_interface node',
            ),
            norbit_fls_ros_interface_node,
            sonar_overlay_node,
            image_to_gstreamer_node,
        ]
    )

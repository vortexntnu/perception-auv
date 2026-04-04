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
        package="image_to_gstreamer",
        executable="image_to_gstreamer_node",
        name="image_to_gstreamer_node",
        additional_env={"EGL_PLATFORM": "surfaceless"},
        parameters=[
            {
                "input_topic": "/fls_image/display_mono",
                "host": LaunchConfiguration("gst_host"),
                "port": LaunchConfiguration("gst_port"),
                "bitrate": LaunchConfiguration("gst_bitrate"),
                "framerate": LaunchConfiguration("gst_framerate"),
                "preset_level": LaunchConfiguration("gst_preset_level"),
                "iframe_interval": LaunchConfiguration("gst_iframe_interval"),
                "control_rate": LaunchConfiguration("gst_control_rate"),
                "pt": LaunchConfiguration("gst_pt"),
                "config_interval": LaunchConfiguration("gst_config_interval"),
                "format": "GRAY8",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the norbit_fls_ros_interface node',
            ),
            DeclareLaunchArgument(
                "gst_host",
                default_value="10.0.0.68",
                description="GStreamer stream destination host",
            ),
            DeclareLaunchArgument(
                "gst_port",
                default_value="5002",
                description="GStreamer stream destination port",
            ),
            DeclareLaunchArgument(
                "gst_bitrate",
                default_value="500000",
                description="GStreamer encoder bitrate (bps)",
            ),
            DeclareLaunchArgument(
                "gst_framerate",
                default_value="15",
                description="GStreamer stream framerate",
            ),
            DeclareLaunchArgument(
                "gst_preset_level",
                default_value="1",
                description="GStreamer encoder preset level",
            ),
            DeclareLaunchArgument(
                "gst_iframe_interval",
                default_value="15",
                description="GStreamer I-frame interval",
            ),
            DeclareLaunchArgument(
                "gst_control_rate",
                default_value="1",
                description="GStreamer control rate",
            ),
            DeclareLaunchArgument(
                "gst_pt",
                default_value="96",
                description="GStreamer RTP payload type",
            ),
            DeclareLaunchArgument(
                "gst_config_interval",
                default_value="1",
                description="GStreamer config interval",
            ),
            norbit_fls_ros_interface_node,
            sonar_overlay_node,
            image_to_gstreamer_node,
        ]
    )

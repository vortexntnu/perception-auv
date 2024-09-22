import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    enable_filtering = LaunchConfiguration("enable_filtering")
    enable_filtering_arg = DeclareLaunchArgument(
        "enable_filtering",
        default_value="True",
        description="enable image filtering",
    )

    enable_aruco = LaunchConfiguration("enable_aruco")
    enable_aruco_arg = DeclareLaunchArgument(
        "enable_aruco",
        default_value="True",
        description="enable Aruco detection",
    )

    enable_gripper_camera = LaunchConfiguration("enable_gripper_camera")
    enable_gripper_camera_arg = DeclareLaunchArgument(
        "enable_gripper_camera",
        default_value="True",
        description="enable gripper camera",
    )

    enable_front_camera = LaunchConfiguration("enable_front_camera")
    enable_front_camera_arg = DeclareLaunchArgument(
        "enable_front_camera",
        default_value="True",
        description="enable front camera",
    )

    enable_composable_nodes = LaunchConfiguration("enable_composable_nodes")
    enable_composable_nodes_arg = DeclareLaunchArgument(
        "enable_composable_nodes",
        default_value="True",
        description="enable composable nodes",
    )

    # Note: Params file for composable node is different format from params file for standard node.
    # However, to make the files compatible for both we use /** as namespace.
    # See https://github.com/ros2/launch_ros/pull/259 for more info.

    filtering_params_file = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "image_filtering_params.yaml",
    )

    aruco_params_file = os.path.join(
        get_package_share_directory("perception_setup"),
        "config",
        "aruco_detector_params.yaml",
    )

    gripper_params_file = PathJoinSubstitution(
        [FindPackageShare("perception_setup"), "config", "gripper_camera_params.yaml"]
    )

    front_camera_params_file = PathJoinSubstitution(
        [FindPackageShare("perception_setup"), "config", "front_camera_params.yaml"]
    )

    blackfly_s_config_file = PathJoinSubstitution(
        [FindPackageShare("perception_setup"), "config", "blackfly_s_params.yaml"]
    )

    package_share_directory = get_package_share_directory("perception_setup")
    gripper_camera_calib_path = os.path.join(
        package_share_directory, "config", "gripper_camera_calib_downscale.yaml"
    )
    # gripper_camera_calib_path = os.path.join(package_share_directory, 'config', 'gripper_camera_calib.yaml')
    front_camera_calib_path = os.path.join(
        package_share_directory, "config", "front_camera_calib_downscale.yaml"
    )

    # Add 'file://' prefix to the path required by the CameraInfoManager that sets the camera calibration in the spinnaker driver
    gripper_camera_calib_url = f"file://{gripper_camera_calib_path}"
    front_camera_calib_url = f"file://{front_camera_calib_path}"

    composable_node_container = ComposableNodeContainer(
        name="perception_image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="image_filtering",
                plugin="vortex::image_processing::ImageFilteringNode",
                name="image_filtering",
                parameters=[filtering_params_file],
                extra_arguments=[{"use_intra_process_comms": False}],
                condition=IfCondition(enable_filtering),
            ),
            ComposableNode(
                package="aruco_detector",
                plugin="vortex::aruco_detector::ArucoDetectorNode",
                name="aruco_detector",
                parameters=[aruco_params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(enable_aruco),
            ),
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name="gripper_camera",
                parameters=[
                    gripper_params_file,
                    {
                        "parameter_file": blackfly_s_config_file,
                        "serial_number": "23494258",
                        "camerainfo_url": gripper_camera_calib_url,
                    },
                ],
                remappings=[
                    ("~/control", "/exposure_control/control"),
                    ("/flir_camera/image_raw", "/gripper_camera/image_raw"),
                    ("/flir_camera/camera_info", "/gripper_camera/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
                condition=IfCondition(enable_gripper_camera),
            ),
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name="front_camera",
                parameters=[
                    front_camera_params_file,
                    {
                        "parameter_file": blackfly_s_config_file,
                        "serial_number": "23494259",
                        "camerainfo_url": front_camera_calib_url,
                    },
                ],
                remappings=[
                    ("~/control", "/exposure_control/control"),
                    ("/flir_camera/image_raw", "/front_camera/image_raw"),
                    ("/flir_camera/camera_info", "/front_camera/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
                condition=IfCondition(enable_front_camera),
            ),
        ],
        output="screen",
        condition=IfCondition(enable_composable_nodes),
    )

    # Create separate nodes for each component to launch them individually if composable nodes are disabled
    filtering_node = Node(
        package="image_filtering",
        executable="image_filtering_node",
        name="image_filtering",
        parameters=[filtering_params_file],
        output="screen",
        condition=IfCondition(enable_filtering),
    )

    aruco_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node",
        name="aruco_detector",
        parameters=[aruco_params_file],
        output="screen",
        condition=IfCondition(enable_aruco),
    )

    gripper_camera_node = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        name="gripper_camera",
        parameters=[
            gripper_params_file,
            {
                "parameter_file": blackfly_s_config_file,
                "serial_number": "23494258",
                "camerainfo_url": gripper_camera_calib_url,
            },
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
            ("/flir_camera/image_raw", "/gripper_camera/image_raw"),
            ("/flir_camera/camera_info", "/gripper_camera/camera_info"),
        ],
        output="screen",
        condition=IfCondition(enable_gripper_camera),
    )

    front_camera_node = Node(
        package="spinnaker_camera_driver",
        executable="camera_driver_node",
        name="front_camera",
        parameters=[
            front_camera_params_file,
            {
                "parameter_file": blackfly_s_config_file,
                "serial_number": "23494259",
                "camerainfo_url": front_camera_calib_url,
            },
        ],
        remappings=[
            ("~/control", "/exposure_control/control"),
            ("/flir_camera/image_raw", "/front_camera/image_raw"),
            ("/flir_camera/camera_info", "/front_camera/camera_info"),
        ],
        output="screen",
        condition=IfCondition(enable_front_camera),
    )

    separate_nodes_group = GroupAction(
        actions=[
            filtering_node,
            aruco_node,
            gripper_camera_node,
            front_camera_node,
        ],
        condition=UnlessCondition(enable_composable_nodes),
    )

    return LaunchDescription(
        [
            enable_filtering_arg,
            enable_aruco_arg,
            enable_gripper_camera_arg,
            enable_front_camera_arg,
            enable_composable_nodes_arg,
            composable_node_container,
            separate_nodes_group,
        ]
    )


generate_launch_description()

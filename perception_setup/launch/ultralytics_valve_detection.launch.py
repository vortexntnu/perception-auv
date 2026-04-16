#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def validate_device(device: str):
    if device in ('cpu', 'cuda', 'mps'):
        return
    if device.isdigit():
        return
    if device.startswith('cuda:') and device.split(':')[1].isdigit():
        return
    raise RuntimeError(
        f"Invalid device '{device}'. Use 'cpu', GPU index (0,1,...), "
        "'cuda', 'cuda:N', or 'mps'."
    )


DETECTIONS_TOPIC = '/ultralytics_valve_detection/detections'


def launch_setup(context, *args, **kwargs):
    device = LaunchConfiguration('device').perform(context)
    validate_device(device)

    yolo_params = os.path.join(
        get_package_share_directory('yolo_obb_object_detection'),
        'config/yolo_obb_object_detection_params.yaml',
    )
    valve_params = os.path.join(
        get_package_share_directory('valve_detection'),
        'config',
        'valve_detection_params.yaml',
    )

    # --- YOLO OBB detection node ---
    yolo_node = Node(
        package='yolo_obb_object_detection',
        executable='yolo_obb_object_detection_node.py',
        name='yolo_obb_object_detection',
        output='screen',
        parameters=[
            yolo_params,
            {
                'device': device,
                'yolo_model': LaunchConfiguration('yolo_model'),
                'model_conf': LaunchConfiguration('model_conf'),
                'color_image_sub_topic': LaunchConfiguration('color_image_sub_topic'),
                'yolo_detections_pub_topic': DETECTIONS_TOPIC,
            },
        ],
    )

    # --- Valve detection composable node ---
    valve_container = ComposableNodeContainer(
        name='valve_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='valve_detection',
                plugin='valve_detection::ValvePoseNode',
                name='valve_pose_node',
                parameters=[
                    valve_params,
                    {
                        'detections_sub_topic': DETECTIONS_TOPIC,
                        'yolo_img_width': 1920,
                        'yolo_img_height': 1080,
                        'depth_image_sub_topic': LaunchConfiguration(
                            'depth_image_sub_topic'
                        ),
                        'depth_image_info_topic': LaunchConfiguration(
                            'depth_image_info_topic'
                        ),
                        'color_image_info_topic': LaunchConfiguration(
                            'color_image_info_topic'
                        ),
                        'depth_frame_id': LaunchConfiguration('depth_frame_id'),
                        'color_frame_id': LaunchConfiguration('color_frame_id'),
                        'landmarks_pub_topic': LaunchConfiguration(
                            'landmarks_pub_topic'
                        ),
                        'output_frame_id': LaunchConfiguration('output_frame_id'),
                        'drone': LaunchConfiguration('drone'),
                        'undistort_detections': LaunchConfiguration(
                            'undistort_detections'
                        ),
                        'debug_visualize': LaunchConfiguration('debug_visualize'),
                        'clamp_rotation': LaunchConfiguration('clamp_rotation'),
                        'use_hardcoded_extrinsic': LaunchConfiguration(
                            'use_hardcoded_extrinsic'
                        ),
                        'extrinsic_tx': LaunchConfiguration('extrinsic_tx'),
                        'extrinsic_ty': LaunchConfiguration('extrinsic_ty'),
                        'extrinsic_tz': LaunchConfiguration('extrinsic_tz'),
                    },
                ],
            )
        ],
        output='screen',
    )

    return [yolo_node, valve_container]


def generate_launch_description():
    return LaunchDescription(
        [
            # --- YOLO arguments ---
            DeclareLaunchArgument(
                'device',
                default_value='0',
                description="YOLO device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
            ),
            DeclareLaunchArgument(
                'yolo_model',
                default_value='/home/vortex/workspaces/isaac_ros-dev/src/perception-auv/perception_setup/models/obb_best_simulator.pt',
                description='YOLO model file name',
            ),
            DeclareLaunchArgument(
                'model_conf',
                default_value='0.4',
                description='YOLO confidence threshold',
            ),
            DeclareLaunchArgument(
                'color_image_sub_topic',
                default_value='/nautilus/front_camera/image_color',
                description='Input color image topic for YOLO',
            ),
            # --- Valve detection arguments ---
            DeclareLaunchArgument(
                'depth_image_sub_topic',
                default_value='/nautilus/depth_camera/image_depth',
                description='Depth image topic',
            ),
            DeclareLaunchArgument(
                'depth_image_info_topic',
                default_value='/nautilus/depth_camera/camera_info',
                description='Depth camera info topic',
            ),
            DeclareLaunchArgument(
                'color_image_info_topic',
                default_value='/nautilus/front_camera/camera_info',
                description='Color camera info topic',
            ),
            DeclareLaunchArgument(
                'depth_frame_id',
                default_value='front_camera_depth_optical',
                description='Depth camera optical frame ID (without drone prefix)',
            ),
            DeclareLaunchArgument(
                'color_frame_id',
                default_value='front_camera_color_optical',
                description='Color camera optical frame ID (without drone prefix)',
            ),
            DeclareLaunchArgument(
                'landmarks_pub_topic',
                default_value='/nautilus/landmarks',
                description='Output valve landmarks topic',
            ),
            DeclareLaunchArgument(
                'output_frame_id',
                default_value='front_camera_depth_optical',
                description='Output frame ID for published poses (without drone prefix)',
            ),
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs',
            ),
            DeclareLaunchArgument(
                'undistort_detections',
                default_value='false',
                description='Undistort bounding-box detections using color camera distortion',
            ),
            DeclareLaunchArgument(
                'debug_visualize',
                default_value='true',
                description='Enable valve detection debug visualization topics',
            ),
            DeclareLaunchArgument(
                'clamp_rotation',
                default_value='true',
                description='Clamp valve handle angle to 0-90 deg (0=vertical, 90=horizontal)',
            ),
            DeclareLaunchArgument(
                'use_hardcoded_extrinsic',
                default_value='true',
                description='Use hardcoded depth-to-color extrinsic instead of TF lookup',
            ),
            DeclareLaunchArgument(
                'extrinsic_tx',
                default_value='-0.059',
                description='Hardcoded extrinsic translation X (metres)',
            ),
            DeclareLaunchArgument(
                'extrinsic_ty',
                default_value='0.0',
                description='Hardcoded extrinsic translation Y (metres)',
            ),
            DeclareLaunchArgument(
                'extrinsic_tz',
                default_value='0.0',
                description='Hardcoded extrinsic translation Z (metres)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )

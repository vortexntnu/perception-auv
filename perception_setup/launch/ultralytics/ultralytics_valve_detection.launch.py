#!/usr/bin/env python3
"""Simulator -> Ultralytics YOLO OBB (Python) -> Valve Detection -> Subtype Resolver.

All topics and TF frames are defined inline in this launch file.
Node tuning defaults live in each node's own yaml under its package
and are loaded once (no inline re-overriding).
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _validate_device(device: str) -> None:
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


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')

    model_file_path = os.path.join(models_dir, 'simulator_valve_handle_large.pt')
    device = LaunchConfiguration('device').perform(context)
    _validate_device(device)

    drone = LaunchConfiguration('drone')
    clamp_yaw = LaunchConfiguration('clamp_yaw')

    detections_topic = '/ultralytics_valve_detection/detections'
    raw_landmarks_topic = '/nautilus/landmarks'
    typed_landmarks_topic = '/nautilus/landmarks_test'

    yolo_node = Node(
        package='yolo_obb_object_detection',
        executable='yolo_obb_object_detection_node',
        name='yolo_obb_object_detection',
        output='screen',
        parameters=[
            {
                'device': device,
                'model_path': model_file_path,
                'confidence_threshold': 0.6,
                'input_topic': '/nautilus/front_camera/image_color',
                'output_detections_topic': detections_topic,
                'output_annotated_topic': '/ultralytics_valve_detection/annotated_image',
            },
        ],
    )

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
                    os.path.join(
                        get_package_share_directory('valve_detection'),
                        'config',
                        'valve_detection_params.yaml',
                    ),
                    {
                        'detections_sub_topic': detections_topic,
                        'depth_image_sub_topic': '/nautilus/depth_camera/image_depth',
                        'depth_image_info_topic': '/nautilus/depth_camera/camera_info',
                        'color_image_info_topic': '/nautilus/front_camera/camera_info',
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': raw_landmarks_topic,
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': drone,
                        'undistort_detections': False,
                        # yolo_obb_object_detection publishes detections in
                        # original image coordinates, not letterbox space.
                        'detections_letterboxed': False,
                        'debug_visualize': True,
                        'use_hardcoded_extrinsic': False,
                        'clamp_yaw': True,
                    },
                ],
            )
        ],
        output='screen',
    )

    subtype_resolver_node = Node(
        package='valve_subtype_resolver',
        executable='valve_subtype_resolver_node',
        name='valve_subtype_resolver_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('valve_subtype_resolver'),
                'config',
                'valve_subtype_resolver_params.yaml',
            ),
            {
                'drone': drone,
                'clamp_yaw': clamp_yaw,
                'landmarks_sub_topic': raw_landmarks_topic,
                'landmarks_pub_topic': typed_landmarks_topic,
            },
        ],
    )

    return [yolo_node, valve_container, subtype_resolver_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'device',
                default_value='0',
                description="YOLO device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
            ),
            DeclareLaunchArgument(
                'drone',
                default_value='nautilus',
                description='Robot name, prepended to TF frame IDs',
            ),
            DeclareLaunchArgument(
                'clamp_yaw',
                default_value='true',
                description=(
                    'Fold landmark yaw into [0, 90°] in the world frame '
                    '(drone-roll invariant). Valve handle is 180°/90° symmetric.'
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

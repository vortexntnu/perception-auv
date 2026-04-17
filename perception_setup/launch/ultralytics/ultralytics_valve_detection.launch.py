#!/usr/bin/env python3
"""Simulator -> Ultralytics YOLO OBB (Python) -> Valve Detection.

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

    model_file_path = os.path.join(models_dir, 'obb_best_simulator.pt')
    device = LaunchConfiguration('device').perform(context)
    _validate_device(device)

    yolo_node = Node(
        package='yolo_obb_object_detection',
        executable='yolo_obb_object_detection_node.py',
        name='yolo_obb_object_detection',
        output='screen',
        parameters=[
            {
                'device': device,
                'yolo_model': model_file_path,
                'model_conf': 0.4,
                'color_image_sub_topic': '/nautilus/front_camera/image_color',
                'yolo_detections_pub_topic': '/ultralytics_valve_detection/detections',
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
                    {
                        'detections_sub_topic': '/ultralytics_valve_detection/detections',
                        'yolo_img_width': 1920,
                        'yolo_img_height': 1080,
                        'depth_image_sub_topic': '/nautilus/depth_camera/image_depth',
                        'depth_image_info_topic': '/nautilus/depth_camera/camera_info',
                        'color_image_info_topic': '/nautilus/front_camera/camera_info',
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': '/nautilus/landmarks',
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': 'nautilus',
                        'undistort_detections': False,
                        'debug_visualize': True,
                        'clamp_rotation': True,
                        'use_hardcoded_extrinsic': True,
                        'extrinsic_tx': -0.059,
                        'extrinsic_ty': 0.0,
                        'extrinsic_tz': 0.0,
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
            DeclareLaunchArgument(
                'device',
                default_value='0',
                description="YOLO device: 'cpu', GPU index, 'cuda', 'cuda:N', or 'mps'",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

"""Valve intervention pipeline launch.

Starts:
  - Camera driver        (realsense_d555 by default, configured via valve_intervention.yaml)
  - Image processing     (undistort / crop, if enabled in cameras.yaml)
  - yolo_obb pipeline    (input: camera image -> output: OBB detections)
  - valve_detection node (input: detections + depth + camera_info -> output: valve landmarks)

All tunable parameters live in perception_setup/config/valve_intervention.yaml.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flatten(d, parent_key=''):
    """Recursively flatten a nested dict to dot-notation ROS parameter keys."""
    items = {}
    for k, v in d.items():
        full_key = f'{parent_key}.{k}' if parent_key else k
        if isinstance(v, dict):
            items.update(_flatten(v, full_key))
        else:
            items[full_key] = v
    return items


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')

    config_path = LaunchConfiguration('config_file').perform(context)
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)

    camera_key = cfg['camera']
    cam = cameras[camera_key]

    actions = []

    # --- Camera driver (skipped when use_rosbag: true) ---
    if cam.get('use_rosbag', False):
        # When using a rosbag, launch undistort/crop separately if enabled
        # (the camera driver launch bundles them, but we skip the driver).
        if cam.get('enable_undistort', False):
            # Undistort needs calibration camera_info — launch the publisher.
            if 'calibration_file' in cam:
                calib_path = os.path.join(
                    pkg_dir, 'config', 'cameras', cam['calibration_file']
                )
                actions.append(
                    Node(
                        package='perception_setup',
                        executable='camera_info_publisher.py',
                        name='camera_info_publisher',
                        parameters=[
                            {
                                'camera_info_file': calib_path,
                                'camera_info_topic': cam[
                                    'calibration_camera_info_topic'
                                ],
                            }
                        ],
                        output='screen',
                    )
                )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg_dir,
                            'launch',
                            'image_processing',
                            'image_undistort.launch.py',
                        )
                    )
                )
            )
        if cam.get('enable_crop', False):
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg_dir,
                            'launch',
                            'image_processing',
                            'image_crop.launch.py',
                        )
                    )
                )
            )
    else:
        # Camera driver launch includes undistort/crop if configured
        camera_launch_path = os.path.join(
            pkg_dir, 'launch', 'cameras', f'{camera_key}.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_path))
        )

    # --- Resolve topics based on enable_undistort (independent of use_rosbag) ---
    if cam.get('enable_undistort', True):
        color_image_topic = cam['image_topic']
        color_info_topic = cam['camera_info_topic']
    else:
        color_image_topic = cam['raw_image_topic']
        color_info_topic = cam['raw_camera_info_topic']

    if cam.get('enable_crop', True):
        depth_image_topic = cam.get('depth_image_topic', cam.get('raw_depth_topic', ''))
        depth_info_topic = cam.get(
            'depth_camera_info_topic', cam.get('raw_depth_camera_info_topic', '')
        )
    else:
        depth_image_topic = cam.get('raw_depth_topic', '')
        depth_info_topic = cam.get('raw_depth_camera_info_topic', '')

    # --- YOLO OBB pipeline ---
    yolo_cfg = dict(cfg['yolo_obb'])
    yolo_cfg['image_input_topic'] = color_image_topic
    yolo_cfg['camera_info_input_topic'] = color_info_topic
    yolo_cfg['input_image_width'] = cam['image_width']
    yolo_cfg['input_image_height'] = cam['image_height']

    tmp_yolo_cfg = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='yolo_obb_', delete=False
    )
    yaml.dump(yolo_cfg, tmp_yolo_cfg, default_flow_style=False)
    tmp_yolo_cfg.close()

    yolo_obb_launch_path = os.path.join(pkg_dir, 'launch', 'yolo', 'yolo_obb.launch.py')
    yolo_obb_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_obb_launch_path),
        launch_arguments={
            'config_file': tmp_yolo_cfg.name,
            'camera': camera_key,
        }.items(),
    )

    # --- Valve Detection node ---
    vd = cfg['valve_detection']

    valve_params = {
        'detections_sub_topic': str(cfg['yolo_obb']['detection_topic']),
        'color_image_info_topic': str(vd['color_image_info_topic']),
        'depth_image_sub_topic': depth_image_topic,
        'depth_image_info_topic': depth_info_topic,
        # Extrinsic frame IDs (from cameras.yaml, looked up from /tf_static)
        'depth_frame_id': str(cam.get('depth_frame_id', '')),
        'color_frame_id': str(cam.get('color_frame_id', '')),
        # Output
        'landmarks_pub_topic': str(vd['landmarks_pub_topic']),
        # YOLO letterbox
        'yolo_img_width': int(vd['yolo_img_width']),
        'yolo_img_height': int(vd['yolo_img_height']),
        # Annulus and plane fit
        'annulus_radius_ratio': float(vd['annulus_radius_ratio']),
        'plane_ransac_threshold': float(vd['plane_ransac_threshold']),
        'plane_ransac_max_iterations': int(vd['plane_ransac_max_iterations']),
        # Duplicate suppression
        'iou_duplicate_threshold': float(vd['iou_duplicate_threshold']),
        # Pose offset
        'valve_handle_offset': float(vd['valve_handle_offset']),
        # Output frame (defaults to depth_frame_id from cameras.yaml)
        'output_frame_id': str(cam.get('depth_frame_id', '')),
        'drone': str(vd['drone']),
        # Debug
        'debug_visualize': bool(vd['debug_visualize']),
    }

    # Add debug params (flattened with dot notation)
    valve_params.update(_flatten(vd['debug'], 'debug'))

    valve_detection_node = Node(
        package='valve_detection',
        executable='valve_detection_node',
        name='valve_detection_node',
        parameters=[valve_params],
        output='screen',
    )

    actions += [yolo_obb_launch, valve_detection_node]
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'valve_intervention.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to valve intervention config YAML',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

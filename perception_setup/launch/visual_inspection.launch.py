"""Visual inspection pipeline launch.

Starts:
  - Camera driver        (realsense_d555 or bfs_pge_16s2c, configured via visual_inspection.yaml)
  - image_filtering_node (input: camera image -> output: filtered image)
  - aruco_detector_node  (input: filtered image -> output: marker poses / board pose)

All tunable parameters live in perception_setup/config/visual_inspection.yaml.
"""

import os

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

    # Resolve image and camera_info topics based on enable_undistort.
    if cam.get('enable_undistort', True):
        image_topic = cam['image_topic']
        camera_info_topic = cam['camera_info_topic']
    else:
        image_topic = cam['raw_image_topic']
        camera_info_topic = cam['raw_camera_info_topic']

    # --- Camera driver / image processing ---
    actions = []
    if cam.get('use_rosbag', False):
        # Rosbag provides raw topics; launch undistort/crop separately if enabled.
        if cam.get('enable_undistort', False):
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
        # Camera driver launch includes undistort/crop if configured.
        camera_launch_path = os.path.join(
            pkg_dir, 'launch', 'cameras', f'{camera_key}.launch.py'
        )
        actions.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch_path))
        )

    # --- image_filtering ---
    filt = cfg['image_filtering']
    filtering_params = {
        'sub_topic': image_topic,
        'pub_topic': str(filt['pub_topic']),
        'input_encoding': str(filt['input_encoding']),
        'output_encoding': str(filt['output_encoding']),
    }
    filtering_params.update(_flatten(filt['filter_params'], 'filter_params'))

    image_filtering_node = Node(
        package='image_filtering',
        executable='image_filtering_node',
        name='image_filtering_node',
        parameters=[filtering_params],
        output='screen',
    )

    # --- aruco_detector ---
    aruco = cfg['aruco_detector']
    aruco_params = {
        'subs.image_topic': str(filt['pub_topic']),
        'subs.camera_info_topic': camera_info_topic,
        'detect_board': bool(aruco['detect_board']),
        'visualize': bool(aruco['visualize']),
        'log_markers': bool(aruco['log_markers']),
        'publish_detections': bool(aruco['publish_detections']),
        'publish_landmarks': bool(aruco['publish_landmarks']),
        'logger_service_name': str(aruco['logger_service_name']),
        'pubs.aruco_image': str(aruco['pubs']['aruco_image']),
        'pubs.aruco_poses': str(aruco['pubs']['aruco_poses']),
        'pubs.board_pose': str(aruco['pubs']['board_pose']),
        'pubs.landmarks': str(aruco['pubs']['landmarks']),
        'aruco.marker_size': float(aruco['aruco']['marker_size']),
        'aruco.dictionary': str(aruco['aruco']['dictionary']),
        'board.xDist': float(aruco['board']['xDist']),
        'board.yDist': float(aruco['board']['yDist']),
        'board.ids': list(aruco['board']['ids']),
    }

    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        parameters=[aruco_params],
        output='screen',
    )

    actions += [image_filtering_node, aruco_detector_node]
    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'visual_inspection.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to visual inspection config YAML',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

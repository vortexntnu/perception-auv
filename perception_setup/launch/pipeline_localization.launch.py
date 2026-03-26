# SPDX-License-Identifier: MIT

"""Pipeline-localization launch: Dual-camera segmentation.

Runs two independent YOLO segmentation pipelines on different camera streams
(front + down). Each pipeline gets its own container and unique topic namespace
to avoid node/topic collisions.

Stage 1 – Front camera segmentation
  /cam/image_color → seg pipeline → /localization/front/segmentation_mask

Stage 2 – Down camera segmentation
  /cam_down/image_color → seg pipeline → /localization/down/segmentation_mask
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _build_seg_pipeline(cfg, prefix, models_dir, encoder_dir):
    """Build a complete segmentation pipeline with unique names.

    Args:
        cfg: Config dict for this segmentation instance.
        prefix: Unique prefix string (e.g. 'front', 'down') used to
                differentiate container names, node names, and internal topics.
        models_dir: Absolute path to the models directory.
        encoder_dir: Package share directory for isaac_ros_dnn_image_encoder.

    Returns:
        List of launch actions for this pipeline.
    """
    # Unique internal topics per pipeline
    converted_image_topic = f'/yolo_seg_{prefix}/internal/converted_image'
    encoder_resize_topic = f'/yolo_seg_{prefix}_encoder/internal/resize/image'
    tensor_output_topic = f'/yolo_seg_{prefix}/tensor_pub'
    tensor_input_topic = f'/yolo_seg_{prefix}/tensor_sub'
    encoder_namespace = f'yolo_seg_{prefix}_encoder/internal'
    container_name = f'seg_{prefix}_tensor_rt_container'

    model_path = os.path.join(models_dir, str(cfg['model_file_path']))
    engine_path = os.path.join(models_dir, str(cfg['engine_file_path']))

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name=f'seg_{prefix}_image_format_converter',
        parameters=[
            {
                'encoding_desired': str(cfg['encoding_desired']),
                'image_width': int(cfg['input_image_width']),
                'image_height': int(cfg['input_image_height']),
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', str(cfg['image_input_topic'])),
            ('image', converted_image_topic),
        ],
    )

    tensor_rt = ComposableNode(
        name=f'seg_{prefix}_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': model_path,
                'engine_file_path': engine_path,
                'output_binding_names': cfg['output_binding_names'],
                'output_tensor_names': cfg['output_tensor_names'],
                'input_tensor_names': cfg['input_tensor_names'],
                'input_binding_names': cfg['input_binding_names'],
                'verbose': bool(cfg['verbose']),
                'force_engine_update': bool(cfg['force_engine_update']),
                'tensor_output_topic': tensor_output_topic,
            }
        ],
        remappings=[
            ('tensor_pub', tensor_output_topic),
            ('tensor_sub', tensor_input_topic),
        ],
    )

    decoder = ComposableNode(
        name=f'seg_{prefix}_decoder',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': tensor_input_topic,
                'confidence_threshold': float(cfg['confidence_threshold']),
                'num_detections': int(cfg['num_detections']),
                'mask_width': int(cfg['mask_width']),
                'mask_height': int(cfg['mask_height']),
                'num_protos': int(cfg['num_protos']),
                'network_image_width': int(cfg['network_image_width']),
                'network_image_height': int(cfg['network_image_height']),
                'output_mask_width': int(cfg['output_mask_width']),
                'output_mask_height': int(cfg['output_mask_height']),
                'detections_topic': str(cfg['detection_topic']),
                'mask_topic': str(cfg['mask_topic']),
            }
        ],
    )

    container = ComposableNodeContainer(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt,
            decoder,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(cfg['input_image_width']),
            'input_image_height': str(cfg['input_image_height']),
            'network_image_width': str(cfg['network_image_width']),
            'network_image_height': str(cfg['network_image_height']),
            'image_mean': str(cfg['image_mean']),
            'image_stddev': str(cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': container_name,
            'dnn_image_encoder_namespace': encoder_namespace,
            'image_input_topic': converted_image_topic,
            'camera_info_input_topic': str(cfg['camera_info_input_topic']),
            'tensor_output_topic': tensor_output_topic,
        }.items(),
    )

    actions = [container, encoder_launch]

    if bool(cfg.get('enable_visualizer', False)):
        actions.append(
            Node(
                package='isaac_ros_yolov26_seg',
                executable='isaac_ros_yolov26_seg_visualizer.py',
                name=f'seg_{prefix}_visualizer',
                parameters=[
                    {
                        'detections_topic': str(cfg['detection_topic']),
                        'image_topic': encoder_resize_topic,
                        'mask_topic': str(cfg['mask_topic']),
                        'output_image_topic': str(cfg['visualized_image_topic']),
                        'class_names_yaml': str(cfg['class_names']),
                    }
                ],
            )
        )

    return actions


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    # Resolve camera references from cameras.yaml
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)

    for seg_cfg in [cfg['seg_front'], cfg['seg_down']]:
        if 'camera' in seg_cfg:
            cam = cameras[seg_cfg['camera']]
            seg_cfg['image_input_topic'] = cam['image_topic']
            seg_cfg['camera_info_input_topic'] = cam['camera_info_topic']
            seg_cfg['input_image_width'] = cam['image_width']
            seg_cfg['input_image_height'] = cam['image_height']

    actions = []
    actions += _build_seg_pipeline(cfg['seg_front'], 'front', models_dir, encoder_dir)
    actions += _build_seg_pipeline(cfg['seg_down'], 'down', models_dir, encoder_dir)

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'pipeline_localization.yaml')

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to pipeline-localization config YAML',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

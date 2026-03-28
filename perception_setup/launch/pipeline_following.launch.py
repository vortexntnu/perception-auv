# SPDX-License-Identifier: MIT

"""Pipeline-following launch: Segmentation -> Classification.

Runs two TensorRT pipelines in a single launch:

Stage 1 - Segmentation
  camera image -> ImageFormatConverter -> DNNImageEncoder -> TensorRT (seg model)
  -> YoloV26SegDecoder -> binary mask + detections + (optional) visualizer

Stage 2 - Classification
  seg mask -> ImageFormatConverter -> DNNImageEncoder -> TensorRT (cls model)
  -> YoloV26ClsDecoder -> UInt8 class ID + (optional) visualizer

The segmentation mask output feeds directly into the classification input.
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

# Stage 1: Segmentation internal topics
SEG_CONVERTED_IMAGE_TOPIC = '/yolo_seg/internal/converted_image'
SEG_ENCODER_RESIZE_TOPIC = '/yolo_seg_encoder/internal/resize/image'
SEG_TENSOR_OUTPUT_TOPIC = '/yolo_seg/tensor_pub'
SEG_TENSOR_INPUT_TOPIC = '/yolo_seg/tensor_sub'
SEG_ENCODER_NAMESPACE = 'yolo_seg_encoder/internal'

# Stage 2: Classification internal topics
CLS_CONVERTED_IMAGE_TOPIC = '/yolo_cls/internal/converted_image'
CLS_ENCODER_RESIZE_TOPIC = '/yolo_cls_encoder/internal/resize/image'
CLS_TENSOR_OUTPUT_TOPIC = '/yolo_cls/tensor_pub'
CLS_TENSOR_INPUT_TOPIC = '/yolo_cls/tensor_sub'
CLS_ENCODER_NAMESPACE = 'yolo_cls_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    seg = cfg['seg']
    cls = cfg['cls']

    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')

    # Resolve camera references from cameras.yaml
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)

    if 'camera' in seg:
        cam = cameras[seg['camera']]
        if cam.get('enable_undistort', True):
            seg['image_input_topic'] = cam['image_topic']
            seg['camera_info_input_topic'] = cam['camera_info_topic']
        else:
            seg['image_input_topic'] = cam['raw_image_topic']
            seg['camera_info_input_topic'] = cam['raw_camera_info_topic']
        seg['input_image_width'] = cam['image_width']
        seg['input_image_height'] = cam['image_height']

    if 'camera' in cls:
        cam = cameras[cls['camera']]
        if cam.get('enable_undistort', True):
            cls['camera_info_input_topic'] = cam['camera_info_topic']
        else:
            cls['camera_info_input_topic'] = cam['raw_camera_info_topic']

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    # Stage 1 – Segmentation
    seg_model = os.path.join(models_dir, str(seg['model_file_path']))
    seg_engine = os.path.join(models_dir, str(seg['engine_file_path']))

    seg_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='seg_image_format_converter',
        parameters=[
            {
                'encoding_desired': str(seg['encoding_desired']),
                'image_width': int(seg['input_image_width']),
                'image_height': int(seg['input_image_height']),
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', str(seg['image_input_topic'])),
            ('image', SEG_CONVERTED_IMAGE_TOPIC),
        ],
    )

    seg_tensor_rt = ComposableNode(
        name='seg_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': seg_model,
                'engine_file_path': seg_engine,
                'output_binding_names': seg['output_binding_names'],
                'output_tensor_names': seg['output_tensor_names'],
                'input_tensor_names': seg['input_tensor_names'],
                'input_binding_names': seg['input_binding_names'],
                'verbose': bool(seg['verbose']),
                'force_engine_update': bool(seg['force_engine_update']),
                'tensor_output_topic': SEG_TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', SEG_TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', SEG_TENSOR_INPUT_TOPIC),
        ],
    )

    seg_decoder = ComposableNode(
        name='seg_decoder',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': SEG_TENSOR_INPUT_TOPIC,
                'confidence_threshold': float(seg['confidence_threshold']),
                'num_detections': int(seg['num_detections']),
                'mask_width': int(seg['mask_width']),
                'mask_height': int(seg['mask_height']),
                'num_protos': int(seg['num_protos']),
                'network_image_width': int(seg['network_image_width']),
                'network_image_height': int(seg['network_image_height']),
                'output_mask_width': int(seg['output_mask_width']),
                'output_mask_height': int(seg['output_mask_height']),
                'detections_topic': str(seg['detection_topic']),
                'mask_topic': str(seg['mask_topic']),
            }
        ],
    )

    seg_container = ComposableNodeContainer(
        name='seg_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            seg_image_format_converter,
            seg_tensor_rt,
            seg_decoder,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    seg_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(seg['input_image_width']),
            'input_image_height': str(seg['input_image_height']),
            'network_image_width': str(seg['network_image_width']),
            'network_image_height': str(seg['network_image_height']),
            'image_mean': str(seg['image_mean']),
            'image_stddev': str(seg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'seg_tensor_rt_container',
            'dnn_image_encoder_namespace': SEG_ENCODER_NAMESPACE,
            'image_input_topic': SEG_CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': str(seg['camera_info_input_topic']),
            'tensor_output_topic': SEG_TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    # Stage 2 – Classification
    cls_model = os.path.join(models_dir, str(cls['model_file_path']))
    cls_engine = os.path.join(models_dir, str(cls['engine_file_path']))

    cls_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='cls_image_format_converter',
        parameters=[
            {
                'encoding_desired': str(cls['encoding_desired']),
                'image_width': int(cls['input_image_width']),
                'image_height': int(cls['input_image_height']),
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', str(cls['image_input_topic'])),
            ('image', CLS_CONVERTED_IMAGE_TOPIC),
        ],
    )

    cls_tensor_rt = ComposableNode(
        name='cls_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': cls_model,
                'engine_file_path': cls_engine,
                'output_binding_names': cls['output_binding_names'],
                'output_tensor_names': cls['output_tensor_names'],
                'input_tensor_names': cls['input_tensor_names'],
                'input_binding_names': cls['input_binding_names'],
                'verbose': bool(cls['verbose']),
                'force_engine_update': bool(cls['force_engine_update']),
                'tensor_output_topic': CLS_TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', CLS_TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', CLS_TENSOR_INPUT_TOPIC),
        ],
    )

    cls_decoder = ComposableNode(
        name='cls_decoder',
        package='isaac_ros_yolov26_cls',
        plugin='nvidia::isaac_ros::yolov26_cls::YoloV26ClsDecoderNode',
        parameters=[
            {
                'tensor_input_topic': CLS_TENSOR_INPUT_TOPIC,
                'confidence_threshold': float(cls['confidence_threshold']),
                'num_classes': int(cls['num_classes']),
                'class_topic': str(cls['class_topic']),
            }
        ],
    )

    cls_container = ComposableNodeContainer(
        name='cls_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            cls_image_format_converter,
            cls_tensor_rt,
            cls_decoder,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    cls_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(cls['input_image_width']),
            'input_image_height': str(cls['input_image_height']),
            'network_image_width': str(cls['network_image_width']),
            'network_image_height': str(cls['network_image_height']),
            'image_mean': str(cls['image_mean']),
            'image_stddev': str(cls['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'cls_tensor_rt_container',
            'dnn_image_encoder_namespace': CLS_ENCODER_NAMESPACE,
            'image_input_topic': CLS_CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': str(cls['camera_info_input_topic']),
            'tensor_output_topic': CLS_TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    # Assemble actions
    actions = [
        seg_container,
        seg_encoder_launch,
        cls_container,
        cls_encoder_launch,
    ]

    # Segmentation visualizer
    if bool(seg.get('enable_visualizer', False)):
        actions.append(
            Node(
                package='isaac_ros_yolov26_seg',
                executable='isaac_ros_yolov26_seg_visualizer.py',
                name='seg_visualizer',
                parameters=[
                    {
                        'detections_topic': str(seg['detection_topic']),
                        'image_topic': SEG_ENCODER_RESIZE_TOPIC,
                        'mask_topic': str(seg['mask_topic']),
                        'output_image_topic': str(seg['visualized_image_topic']),
                        'class_names_yaml': str(seg['class_names']),
                    }
                ],
            )
        )

    # Classification visualizer
    if bool(cls.get('enable_visualizer', False)):
        actions.append(
            Node(
                package='isaac_ros_yolov26_cls',
                executable='isaac_ros_yolov26_cls_visualizer.py',
                name='cls_visualizer',
                parameters=[
                    {
                        'class_topic': str(cls['class_topic']),
                        'image_topic': CLS_ENCODER_RESIZE_TOPIC,
                        'output_image_topic': str(cls['visualized_image_topic']),
                        'class_names_yaml': str(cls['class_names']),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'pipeline_following.yaml')

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to pipeline-following config YAML',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

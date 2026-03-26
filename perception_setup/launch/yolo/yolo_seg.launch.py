# SPDX-License-Identifier: MIT

"""Isaac ROS YOLO Instance-Segmentation TensorRT inference pipeline

1. ImageFormatConverterNode
   Input:  image_input_topic
   Output: /yolo_seg/internal/converted_image
   Purpose: Convert camera image encoding (e.g. bgra8 -> rgb8)

2. DNNImageEncoderNode
   Input:  /yolo_seg/internal/converted_image
   Output: /tensor_pub
   Purpose: Resize + normalize image and convert to network tensor

3. TensorRTNode
   Input:  /tensor_pub
   Output: /tensor_sub
   Purpose: Run TensorRT inference
   Expected output tensors:
     output0 [1, 300, 38]  detections with mask coefficients
     output1 [1, 32, 160, 160]  prototype masks

4. YoloV26SegDecoderNode
   Input:  /tensor_sub (NitrosTensorList with both output tensors)
   Output: detection_topic  (Detection2DArray)
           mask_topic        (mono8 Image - combined binary mask)
   Purpose: Decode detections and compute instance segmentation mask

5. (Optional) YoloSegVisualizer
   Input:  detection_topic + encoder resize image + mask_topic
   Output: overlay visualization
   Purpose: Draw mask overlay and bounding boxes on image
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

CONVERTED_IMAGE_TOPIC = '/yolo_seg/internal/converted_image'
ENCODER_RESIZE_TOPIC = '/yolo_seg_encoder/internal/resize/image'
TENSOR_OUTPUT_TOPIC = '/yolo_seg/tensor_pub'
TENSOR_INPUT_TOPIC = '/yolo_seg/tensor_sub'
DNN_IMAGE_ENCODER_NAMESPACE = 'yolo_seg_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    pkg_dir = get_package_share_directory('perception_setup')

    # Resolve camera reference from cameras.yaml
    if 'camera' in cfg:
        cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
        with open(cameras_path) as f:
            cameras = yaml.safe_load(f)
        cam = cameras[cfg['camera']]
        cfg['image_input_topic'] = cam['image_topic']
        cfg['camera_info_input_topic'] = cam['camera_info_topic']
        cfg['input_image_width'] = cam['image_width']
        cfg['input_image_height'] = cam['image_height']
    models_dir = os.path.join(pkg_dir, 'models')

    model_file_path = os.path.join(models_dir, str(cfg['model_file_path']))
    engine_file_path = os.path.join(models_dir, str(cfg['engine_file_path']))

    input_tensor_names = cfg['input_tensor_names']
    input_binding_names = cfg['input_binding_names']
    output_tensor_names = cfg['output_tensor_names']
    output_binding_names = cfg['output_binding_names']

    verbose = bool(cfg['verbose'])
    force_engine_update = bool(cfg['force_engine_update'])

    input_image_width = int(cfg['input_image_width'])
    input_image_height = int(cfg['input_image_height'])
    encoding_desired = str(cfg['encoding_desired'])

    network_image_width = int(cfg['network_image_width'])
    network_image_height = int(cfg['network_image_height'])
    image_mean = cfg['image_mean']
    image_stddev = cfg['image_stddev']

    confidence_threshold = float(cfg['confidence_threshold'])
    num_detections = int(cfg['num_detections'])
    mask_width = int(cfg['mask_width'])
    mask_height = int(cfg['mask_height'])
    num_protos = int(cfg['num_protos'])
    output_mask_width = int(cfg['output_mask_width'])
    output_mask_height = int(cfg['output_mask_height'])
    detection_topic = str(cfg['detection_topic'])
    mask_topic = str(cfg['mask_topic'])

    image_input_topic = str(cfg['image_input_topic'])
    camera_info_input_topic = str(cfg['camera_info_input_topic'])

    enable_visualizer = bool(cfg['enable_visualizer'])
    visualized_image_topic = str(cfg['visualized_image_topic'])
    class_names = cfg['class_names']

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': encoding_desired,
                'image_width': input_image_width,
                'image_height': input_image_height,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', image_input_topic),
            ('image', CONVERTED_IMAGE_TOPIC),
        ],
    )

    tensor_rt_node = ComposableNode(
        name='tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': model_file_path,
                'engine_file_path': engine_file_path,
                'output_binding_names': output_binding_names,
                'output_tensor_names': output_tensor_names,
                'input_tensor_names': input_tensor_names,
                'input_binding_names': input_binding_names,
                'verbose': verbose,
                'force_engine_update': force_engine_update,
                'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', TENSOR_INPUT_TOPIC),
        ],
    )

    yolo_seg_decoder_node = ComposableNode(
        name='yolo_seg_decoder_node',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': TENSOR_INPUT_TOPIC,
                'confidence_threshold': confidence_threshold,
                'num_detections': num_detections,
                'mask_width': mask_width,
                'mask_height': mask_height,
                'num_protos': num_protos,
                'network_image_width': network_image_width,
                'network_image_height': network_image_height,
                'output_mask_width': output_mask_width,
                'output_mask_height': output_mask_height,
                'detections_topic': detection_topic,
                'mask_topic': mask_topic,
            }
        ],
    )

    tensor_rt_container = ComposableNodeContainer(
        name='seg_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt_node,
            yolo_seg_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    yolo_seg_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                encoder_dir,
                'launch',
                'dnn_image_encoder.launch.py',
            )
        ),
        launch_arguments={
            'input_image_width': str(input_image_width),
            'input_image_height': str(input_image_height),
            'network_image_width': str(network_image_width),
            'network_image_height': str(network_image_height),
            'image_mean': str(image_mean),
            'image_stddev': str(image_stddev),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'seg_tensor_rt_container',
            'dnn_image_encoder_namespace': DNN_IMAGE_ENCODER_NAMESPACE,
            'image_input_topic': CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': camera_info_input_topic,
            'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    actions = [tensor_rt_container, yolo_seg_encoder_launch]

    if enable_visualizer:
        actions.append(
            Node(
                package='isaac_ros_yolov26_seg',
                executable='isaac_ros_yolov26_seg_visualizer.py',
                name='yolo_seg_visualizer',
                parameters=[
                    {
                        'detections_topic': detection_topic,
                        'image_topic': ENCODER_RESIZE_TOPIC,
                        'mask_topic': mask_topic,
                        'output_image_topic': visualized_image_topic,
                        'class_names_yaml': str(class_names),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to YOLO segmentation pipeline config YAML',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

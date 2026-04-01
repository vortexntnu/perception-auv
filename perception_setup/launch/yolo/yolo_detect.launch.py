# SPDX-License-Identifier: MIT

"""Isaac ROS YOLOv8 TensorRT inference pipeline.

1. ImageFormatConverterNode
   Input:  image_input_topic
   Output: /yolov8/internal/converted_image
   Purpose: Convert camera image to desired encoding (e.g. bgra8 -> rgb8)

2. DNNImageEncoderNode
   Input:  /yolov8/internal/converted_image
   Output: /tensor_pub
   Purpose: Resize + normalize image and convert to network tensor

3. TensorRTNode
   Input:  /tensor_pub
   Output: /tensor_sub
   Purpose: Run TensorRT inference on encoded tensor

4. YoloV8DecoderNode
   Input:  /tensor_sub
   Output: /yolov8/internal/detection_topic
   Purpose: Convert network output tensor -> Detection2DArray

5. (Optional) YOLOv8Visualizer
   Input:  /yolov8/internal/detection_topic + /yolov8/internal/converted_image
   Output: overlay visualization
   Purpose: Draw bounding boxes on image
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

# Internal pipeline topics
CONVERTED_IMAGE_TOPIC = '/yolov8/internal/converted_image'
ENCODER_RESIZE_TOPIC = '/yolov8_encoder/internal/resize/image'
TENSOR_OUTPUT_TOPIC = '/yolov8/tensor_pub'
TENSOR_INPUT_TOPIC = '/yolov8/tensor_sub'
DNN_IMAGE_ENCODER_NAMESPACE = 'yolov8_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_file').perform(context)

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    pkg_dir = get_package_share_directory('perception_setup')

    # Resolve camera topics from cameras.yaml (single source of truth)
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras[LaunchConfiguration('camera').perform(context)]
    image_input_topic = cam['image_topic']
    camera_info_input_topic = cam['camera_info_topic']
    input_image_width = int(cam['image_width'])
    input_image_height = int(cam['image_height'])

    model_file_path = str(cfg['model_file_path'])
    engine_file_path = str(cfg['engine_file_path'])

    input_tensor_names = cfg['input_tensor_names']
    input_binding_names = cfg['input_binding_names']
    output_tensor_names = cfg['output_tensor_names']
    output_binding_names = cfg['output_binding_names']

    verbose = bool(cfg['verbose'])
    force_engine_update = bool(cfg['force_engine_update'])

    encoding_desired = str(cfg['encoding_desired'])

    network_image_width = int(cfg['network_image_width'])
    network_image_height = int(cfg['network_image_height'])
    image_mean = cfg['image_mean']
    image_stddev = cfg['image_stddev']

    confidence_threshold = float(cfg['confidence_threshold'])
    nms_threshold = float(cfg['nms_threshold'])
    num_classes = int(cfg['num_classes'])
    detection_topic = str(cfg['detection_topic'])

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

    yolov8_decoder_node = ComposableNode(
        name='yolov8_decoder_node',
        package='isaac_ros_yolov8',
        plugin='nvidia::isaac_ros::yolov8::YoloV8DecoderNode',
        parameters=[
            {
                'tensor_input_topic': TENSOR_INPUT_TOPIC,
                'confidence_threshold': confidence_threshold,
                'nms_threshold': nms_threshold,
                'num_classes': num_classes,
                'detections_topic': detection_topic,
            }
        ],
    )

    tensor_rt_container = ComposableNodeContainer(
        name='detect_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt_node,
            yolov8_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    yolov8_encoder_launch = IncludeLaunchDescription(
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
            'component_container_name': 'detect_tensor_rt_container',
            'dnn_image_encoder_namespace': DNN_IMAGE_ENCODER_NAMESPACE,
            'image_input_topic': CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': camera_info_input_topic,
            'tensor_output_topic': TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    actions = [
        tensor_rt_container,
        yolov8_encoder_launch,
    ]

    if enable_visualizer:
        actions.append(
            Node(
                package='isaac_ros_yolov8',
                executable='isaac_ros_yolov8_visualizer.py',
                name='yolov8_visualizer',
                parameters=[
                    {
                        'detections_topic': detection_topic,
                        'image_topic': ENCODER_RESIZE_TOPIC,
                        'output_image_topic': visualized_image_topic,
                        'class_names_yaml': str(class_names),
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('perception_setup')
    default_config = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_detect.yaml')

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=default_config,
                description='Path to YOLO pipeline config YAML',
            ),
            DeclareLaunchArgument(
                'camera',
                default_value='realsense_d555',
                description='Camera key in cameras.yaml',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

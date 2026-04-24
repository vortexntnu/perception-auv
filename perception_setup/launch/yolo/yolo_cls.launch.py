# SPDX-License-Identifier: MIT

"""Isaac ROS YOLO classification TensorRT inference pipeline.

1. ImageFormatConverterNode
   Input:  IMAGE_INPUT_TOPIC (launch arg)
   Output: CONVERTED_IMAGE_TOPIC

2. DNNImageEncoderNode
   Input:  CONVERTED_IMAGE_TOPIC
   Output: TENSOR_OUTPUT_TOPIC

3. TensorRTNode
   Input:  TENSOR_OUTPUT_TOPIC
   Output: TENSOR_INPUT_TOPIC
   Expected output tensor shape: [1, num_classes]

4. YoloV26ClsDecoderNode
   Input:  TENSOR_INPUT_TOPIC
   Output: CLASS_TOPIC (UInt8 with predicted class index)
"""

import os

import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')

    model_file_path = os.path.join(models_dir, 'end-of-pipeline-yolov26.onnx')
    engine_file_path = os.path.join(models_dir, 'end-of-pipeline-yolov26.engine')

    config_path = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_cls.yaml')
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 640,
                'image_height': 640,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', '/yolo_seg_mask'),
            ('image', '/yolo_cls/internal/converted_image'),
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
                'output_binding_names': cfg['output_binding_names'],
                'output_tensor_names': cfg['output_tensor_names'],
                'input_tensor_names': cfg['input_tensor_names'],
                'input_binding_names': cfg['input_binding_names'],
                'verbose': True,
                'force_engine_update': False,
                'tensor_output_topic': '/yolo_cls/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/yolo_cls/tensor_pub'),
            ('tensor_sub', '/yolo_cls/tensor_sub'),
        ],
    )

    yolo_cls_decoder_node = ComposableNode(
        name='yolo_cls_decoder_node',
        package='isaac_ros_yolov26_cls',
        plugin='nvidia::isaac_ros::yolov26_cls::YoloV26ClsDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/yolo_cls/tensor_sub',
                'confidence_threshold': float(cfg['confidence_threshold']),
                'num_classes': int(cfg['num_classes']),
                'class_topic': '/classification_output',
            }
        ],
    )

    tensor_rt_container = ComposableNodeContainer(
        name='cls_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt_node,
            yolo_cls_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolo_cls_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(640),
            'input_image_height': str(640),
            'network_image_width': str(cfg['network_image_width']),
            'network_image_height': str(cfg['network_image_height']),
            'image_mean': str(cfg['image_mean']),
            'image_stddev': str(cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'cls_tensor_rt_container',
            'dnn_image_encoder_namespace': 'yolo_cls_encoder/internal',
            'image_input_topic': '/yolo_cls/internal/converted_image',
            'camera_info_input_topic': '/camera/camera/color/camera_info_rect',
            'tensor_output_topic': '/yolo_cls/tensor_pub',
        }.items(),
    )

    actions = [tensor_rt_container, yolo_cls_encoder_launch]

    actions.append(
        Node(
            package='isaac_ros_yolov26_cls',
            executable='isaac_ros_yolov26_cls_visualizer.py',
            name='yolo_cls_visualizer',
            parameters=[
                {
                    'class_topic': '/classification_output',
                    'image_topic': '/dnn_image_encoder/resize/image',
                    'output_image_topic': '/yolo_cls_processed_image',
                    'class_names_yaml': str({0: 'continue', 1: 'end'}),
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=_launch_setup)])

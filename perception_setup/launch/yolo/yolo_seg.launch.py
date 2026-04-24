# SPDX-License-Identifier: MIT

"""Isaac ROS YOLO Instance-Segmentation TensorRT inference pipeline.

1. ImageFormatConverterNode
   Input:  IMAGE_INPUT_TOPIC (launch arg)
   Output: CONVERTED_IMAGE_TOPIC

2. DNNImageEncoderNode
   Input:  CONVERTED_IMAGE_TOPIC
   Output: TENSOR_OUTPUT_TOPIC

3. TensorRTNode
   Input:  TENSOR_OUTPUT_TOPIC
   Output: TENSOR_INPUT_TOPIC
   Expected output tensors:
     output0 [1, 300, 38]  detections with mask coefficients
     output1 [1, 32, 160, 160]  prototype masks

4. YoloV26SegDecoderNode
   Input:  TENSOR_INPUT_TOPIC
   Output: DETECTION_TOPIC (Detection2DArray), MASK_TOPIC (mono8 Image)

5. (Optional) YoloSegVisualizer -> VISUALIZED_IMAGE_TOPIC
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
    config_path = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    model_file_path = os.path.join(models_dir, 'seg_pipe_real.onnx')
    engine_file_path = os.path.join(models_dir, 'seg_pipe_real.engine')

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 720,
                'image_height': 540,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', '/blackfly_s/image_raw'),
            ('image', '/yolo_seg/internal/converted_image'),
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
                'tensor_output_topic': '/yolo_seg/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/yolo_seg/tensor_pub'),
            ('tensor_sub', '/yolo_seg/tensor_sub'),
        ],
    )

    yolo_seg_decoder_node = ComposableNode(
        name='yolo_seg_decoder_node',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/yolo_seg/tensor_sub',
                'confidence_threshold': float(cfg['confidence_threshold']),
                'num_detections': int(cfg['num_detections']),
                'mask_width': int(cfg['mask_width']),
                'mask_height': int(cfg['mask_height']),
                'num_protos': int(cfg['num_protos']),
                'network_image_width': int(cfg['network_image_width']),
                'network_image_height': int(cfg['network_image_height']),
                'output_mask_width': 640,
                'output_mask_height': 640,
                'detections_topic': '/seg_detections_output',
                'mask_topic': '/yolo_seg_mask',
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
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(720),
            'input_image_height': str(540),
            'network_image_width': str(cfg['network_image_width']),
            'network_image_height': str(cfg['network_image_height']),
            'image_mean': str(cfg['image_mean']),
            'image_stddev': str(cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'seg_tensor_rt_container',
            'dnn_image_encoder_namespace': 'yolo_seg_encoder/internal',
            'image_input_topic': '/yolo_seg/internal/converted_image',
            'camera_info_input_topic': '/blackfly_s/camera_info',
            'tensor_output_topic': '/yolo_seg/tensor_pub',
        }.items(),
    )

    actions = [tensor_rt_container, yolo_seg_encoder_launch]

    actions.append(
        Node(
            package='isaac_ros_yolov26_seg',
            executable='isaac_ros_yolov26_seg_visualizer.py',
            name='yolo_seg_visualizer',
            parameters=[
                {
                    'detections_topic': '/seg_detections_output',
                    'image_topic': '/yolo_seg_encoder/internal/resize/image',
                    'mask_topic': '/yolo_seg_mask',
                    'output_image_topic': '/yolo_seg_processed_image',
                    'class_names_yaml': str({0: 'pipeline'}),
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=_launch_setup)])

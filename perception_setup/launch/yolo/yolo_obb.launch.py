# SPDX-License-Identifier: MIT

"""Isaac ROS YOLO-OBB TensorRT inference pipeline.

1. ImageFormatConverterNode
   Input:  IMAGE_INPUT_TOPIC (launch arg)
   Output: CONVERTED_IMAGE_TOPIC

2. DNNImageEncoderNode
   Input:  CONVERTED_IMAGE_TOPIC
   Output: TENSOR_OUTPUT_TOPIC

3. TensorRTNode
   Input:  TENSOR_OUTPUT_TOPIC
   Output: TENSOR_INPUT_TOPIC
   Expected output tensor shape: [1, num_detections, 7]

4. YoloV26OBBDecoderNode
   Input:  TENSOR_INPUT_TOPIC
   Output: DETECTION_TOPIC (Detection2DArray with BoundingBox2D.center.theta = OBB angle)
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
    config_path = os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    model_file_path = os.path.join(models_dir, 'obb_best.onnx')
    engine_file_path = os.path.join(models_dir, 'obb_best.engine')

    image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_converter',
        parameters=[
            {
                'encoding_desired': 'rgb8',
                'image_width': 896,
                'image_height': 504,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', '/realsense_d555/color/image_rect'),
            ('image', '/yolo_obb/internal/converted_image'),
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
                'tensor_output_topic': '/yolo_obb/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/yolo_obb/tensor_pub'),
            ('tensor_sub', '/yolo_obb/tensor_sub'),
        ],
    )

    yolo_obb_decoder_node = ComposableNode(
        name='yolo_obb_decoder_node',
        package='isaac_ros_yolov26_obb',
        plugin='nvidia::isaac_ros::yolov26_obb::YoloV26OBBDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/yolo_obb/tensor_sub',
                'confidence_threshold': float(cfg['confidence_threshold']),
                'num_detections': int(cfg['num_detections']),
                'detections_topic': '/obb_detections_output',
            }
        ],
    )

    tensor_rt_container = ComposableNodeContainer(
        name='obb_tensor_rt_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_converter,
            tensor_rt_node,
            yolo_obb_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
        namespace='',
    )

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    yolo_obb_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(896),
            'input_image_height': str(504),
            'network_image_width': str(cfg['network_image_width']),
            'network_image_height': str(cfg['network_image_height']),
            'image_mean': str(cfg['image_mean']),
            'image_stddev': str(cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'obb_tensor_rt_container',
            'dnn_image_encoder_namespace': 'yolo_obb_encoder/internal',
            'image_input_topic': '/yolo_obb/internal/converted_image',
            'camera_info_input_topic': '/realsense_d555/color/camera_info',
            'tensor_output_topic': '/yolo_obb/tensor_pub',
        }.items(),
    )

    actions = [tensor_rt_container, yolo_obb_encoder_launch]

    actions.append(
        Node(
            package='isaac_ros_yolov26_obb',
            executable='isaac_ros_yolov26_obb_visualizer.py',
            name='yolo_obb_visualizer',
            parameters=[
                {
                    'detections_topic': '/obb_detections_output',
                    'image_topic': '/yolo_obb_encoder/internal/resize/image',
                    'output_image_topic': '/yolo_obb_processed_image',
                    'class_names_yaml': str({0: 'valve'}),
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=_launch_setup)])

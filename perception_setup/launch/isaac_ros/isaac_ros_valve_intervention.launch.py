"""Simulator -> YOLO OBB -> Valve Detection.

Simulator variant of valve_intervention.launch.py. Instead of running a
RealSense camera and image_undistort, this launch file accepts the color
image topic, camera info topic, depth topics, and image dimensions directly
as launch arguments.

Pipeline:
1. Simulator publishes color image, camera info, depth image and depth info
2. YOLO OBB detects oriented bounding boxes on the color image
3. Valve Detection uses oriented bounding boxes + depth image to compute valve pose
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

    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_obb.yaml')) as f:
        yolo_cfg = yaml.safe_load(f)

    model_file_path = os.path.join(models_dir, 'obb_best_simulator.onnx')
    engine_file_path = os.path.join(models_dir, 'obb_best_simulator.engine')

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
            ('image_raw', '/nautilus/front_camera/image_color'),
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
                'output_binding_names': yolo_cfg['output_binding_names'],
                'output_tensor_names': yolo_cfg['output_tensor_names'],
                'input_tensor_names': yolo_cfg['input_tensor_names'],
                'input_binding_names': yolo_cfg['input_binding_names'],
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
                'confidence_threshold': float(yolo_cfg['confidence_threshold']),
                'num_detections': int(yolo_cfg['num_detections']),
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
    dnn_image_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(640),
            'input_image_height': str(640),
            'network_image_width': str(yolo_cfg['network_image_width']),
            'network_image_height': str(yolo_cfg['network_image_height']),
            'image_mean': str(yolo_cfg['image_mean']),
            'image_stddev': str(yolo_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'obb_tensor_rt_container',
            'dnn_image_encoder_namespace': 'yolo_obb_encoder/internal',
            'image_input_topic': '/yolo_obb/internal/converted_image',
            'camera_info_input_topic': '/nautilus/front_camera/camera_info',
            'tensor_output_topic': '/yolo_obb/tensor_pub',
        }.items(),
    )

    yolo_obb_visualizer = Node(
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

    valve_detection_tuning = os.path.join(
        get_package_share_directory('valve_detection'),
        'config',
        'valve_detection_params.yaml',
    )

    valve_detection_container = ComposableNodeContainer(
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
                    valve_detection_tuning,
                    {
                        'depth_image_sub_topic': '/nautilus/depth_camera/image_depth',
                        'detections_sub_topic': '/obb_detections_output',
                        'depth_image_info_topic': '/nautilus/depth_camera/camera_info',
                        'depth_frame_id': 'front_camera_depth_optical',
                        'color_frame_id': 'front_camera_color_optical',
                        'landmarks_pub_topic': '/valve_landmarks',
                        'output_frame_id': 'front_camera_depth_optical',
                        'drone': 'nautilus',
                        'undistort_detections': False,
                        'debug_visualize': True,
                        'clamp_rotation': True,
                        'use_hardcoded_extrinsic': True,
                        'extrinsic_tx': -0.059,
                        'extrinsic_ty': 0.0,
                        'extrinsic_tz': 0.0,
                        'debug.depth_colormap_value_min': 0.1,
                        'debug.depth_colormap_value_max': 2.0,
                    },
                ],
            )
        ],
        output='screen',
    )

    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': '/yolo_obb_processed_image',
                'host': '10.0.0.169',
                'port': 5000,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'RGB',
            }
        ],
        output='screen',
    )

    actions = [
        tensor_rt_container,
        dnn_image_encoder_launch,
        valve_detection_container,
        image_to_gstreamer_node,
        yolo_obb_visualizer,
    ]
    return actions


def generate_launch_description():
    return launch.LaunchDescription([OpaqueFunction(function=_launch_setup)])

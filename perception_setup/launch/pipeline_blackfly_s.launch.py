"""Blackfly S -> YOLO Segmentation -> YOLO Classification -> GStreamer stream.

Pipeline:
1. Blackfly S camera driver publishes raw BGR image
2. YOLO segmentation detects pipeline instances and produces a segmentation mask
   and visualization image
3. YOLO classification runs on the segmentation mask to classify the pipeline state
4. image_to_gstreamer streams the segmentation visualization image over RTP/H.265
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# All pipeline-internal and output topics are prefixed to avoid collisions
# when running alongside other camera pipelines.
CAM_PREFIX = '/down_cam'

# Segmentation pipeline internal topics
SEG_CONVERTED_IMAGE_TOPIC = f'{CAM_PREFIX}/yolo_seg/internal/converted_image'
SEG_TENSOR_OUTPUT_TOPIC = f'{CAM_PREFIX}/yolo_seg/tensor_pub'
SEG_TENSOR_INPUT_TOPIC = f'{CAM_PREFIX}/yolo_seg/tensor_sub'
SEG_ENCODER_NAMESPACE = 'down_cam/yolo_seg_encoder/internal'

# Classification pipeline internal topics
CLS_CONVERTED_IMAGE_TOPIC = f'{CAM_PREFIX}/yolo_cls/internal/converted_image'
CLS_TENSOR_OUTPUT_TOPIC = f'{CAM_PREFIX}/yolo_cls/tensor_pub'
CLS_TENSOR_INPUT_TOPIC = f'{CAM_PREFIX}/yolo_cls/tensor_sub'
CLS_ENCODER_NAMESPACE = 'down_cam/yolo_cls_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')

    # Load camera config
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras['blackfly_s']

    # Load model configs
    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')) as f:
        seg_cfg = yaml.safe_load(f)
    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_cls.yaml')) as f:
        cls_cfg = yaml.safe_load(f)

    models_dir = os.path.join(pkg_dir, 'models')

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    # Prefix yaml-sourced topics
    seg_detection_topic = f'{CAM_PREFIX}{seg_cfg["detection_topic"]}'
    seg_mask_topic = f'{CAM_PREFIX}{seg_cfg["mask_topic"]}'
    seg_visualized_topic = f'{CAM_PREFIX}{seg_cfg["visualized_image_topic"]}'
    cls_image_input_topic = f'{CAM_PREFIX}{cls_cfg["image_input_topic"]}'
    cls_class_topic = f'{CAM_PREFIX}{cls_cfg["class_topic"]}'
    cls_visualized_topic = f'{CAM_PREFIX}{cls_cfg["visualized_image_topic"]}'

    # -------------------------------------------------------------------------
    # Camera
    # -------------------------------------------------------------------------
    spinnaker_map = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_params.yaml')
    driver_params = os.path.join(
        pkg_dir, 'config', 'cameras', 'blackfly_s_driver_params.yaml'
    )
    calib_path = os.path.join(pkg_dir, 'config', 'cameras', 'blackfly_s_calib.yaml')

    camera_container = ComposableNodeContainer(
        name='down_cam_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='blackfly_s',
                parameters=[
                    driver_params,
                    {
                        'parameter_file': spinnaker_map,
                        'serial_number': '23494258',
                        'camerainfo_url': f'file://{calib_path}',
                        'reliable_qos': True,
                    },
                ],
                remappings=[('~/control', '/exposure_control/control')],
                extra_arguments=[{'use_intra_process_comms': True}],
                condition=IfCondition(LaunchConfiguration('enable_camera')),
            ),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # -------------------------------------------------------------------------
    # YOLO Segmentation
    # -------------------------------------------------------------------------
    seg_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='seg_image_format_converter',
        parameters=[
            {
                'encoding_desired': seg_cfg['encoding_desired'],
                'image_width': int(cam['image_width']),
                'image_height': int(cam['image_height']),
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', cam['image_topic']),
            ('image', SEG_CONVERTED_IMAGE_TOPIC),
        ],
    )

    seg_tensor_rt_node = ComposableNode(
        name='seg_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(models_dir, seg_cfg['model_file_path']),
                'engine_file_path': os.path.join(
                    models_dir, seg_cfg['engine_file_path']
                ),
                'output_binding_names': seg_cfg['output_binding_names'],
                'output_tensor_names': seg_cfg['output_tensor_names'],
                'input_tensor_names': seg_cfg['input_tensor_names'],
                'input_binding_names': seg_cfg['input_binding_names'],
                'verbose': bool(seg_cfg['verbose']),
                'force_engine_update': bool(seg_cfg['force_engine_update']),
                'tensor_output_topic': SEG_TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', SEG_TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', SEG_TENSOR_INPUT_TOPIC),
        ],
    )

    seg_decoder_node = ComposableNode(
        name='seg_decoder_node',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': SEG_TENSOR_INPUT_TOPIC,
                'confidence_threshold': float(seg_cfg['confidence_threshold']),
                'num_detections': int(seg_cfg['num_detections']),
                'mask_width': int(seg_cfg['mask_width']),
                'mask_height': int(seg_cfg['mask_height']),
                'num_protos': int(seg_cfg['num_protos']),
                'network_image_width': int(seg_cfg['network_image_width']),
                'network_image_height': int(seg_cfg['network_image_height']),
                'output_mask_width': int(seg_cfg['output_mask_width']),
                'output_mask_height': int(seg_cfg['output_mask_height']),
                'detections_topic': seg_detection_topic,
                'mask_topic': seg_mask_topic,
            }
        ],
    )

    seg_container = ComposableNodeContainer(
        name='down_cam_seg_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            seg_image_format_converter,
            seg_tensor_rt_node,
            seg_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    seg_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(cam['image_width']),
            'input_image_height': str(cam['image_height']),
            'network_image_width': str(seg_cfg['network_image_width']),
            'network_image_height': str(seg_cfg['network_image_height']),
            'image_mean': str(seg_cfg['image_mean']),
            'image_stddev': str(seg_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'down_cam_seg_container',
            'dnn_image_encoder_namespace': SEG_ENCODER_NAMESPACE,
            'image_input_topic': SEG_CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': cam['camera_info_topic'],
            'tensor_output_topic': SEG_TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    seg_visualizer = (
        Node(
            package='isaac_ros_yolov26_seg',
            executable='isaac_ros_yolov26_seg_visualizer.py',
            name='seg_visualizer',
            parameters=[
                {
                    'detections_topic': seg_detection_topic,
                    'image_topic': f'/{SEG_ENCODER_NAMESPACE}/resize/image',
                    'mask_topic': seg_mask_topic,
                    'output_image_topic': seg_visualized_topic,
                    'class_names_yaml': str(seg_cfg['class_names']),
                }
            ],
        )
        if bool(seg_cfg['enable_visualizer'])
        else None
    )

    # -------------------------------------------------------------------------
    # YOLO Classification
    # -------------------------------------------------------------------------
    cls_image_format_converter = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='cls_image_format_converter',
        parameters=[
            {
                'encoding_desired': cls_cfg['encoding_desired'],
                'image_width': int(cls_cfg['input_image_width']),
                'image_height': int(cls_cfg['input_image_height']),
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', cls_image_input_topic),
            ('image', CLS_CONVERTED_IMAGE_TOPIC),
        ],
    )

    cls_tensor_rt_node = ComposableNode(
        name='cls_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(models_dir, cls_cfg['model_file_path']),
                'engine_file_path': os.path.join(
                    models_dir, cls_cfg['engine_file_path']
                ),
                'output_binding_names': cls_cfg['output_binding_names'],
                'output_tensor_names': cls_cfg['output_tensor_names'],
                'input_tensor_names': cls_cfg['input_tensor_names'],
                'input_binding_names': cls_cfg['input_binding_names'],
                'verbose': bool(cls_cfg['verbose']),
                'force_engine_update': bool(cls_cfg['force_engine_update']),
                'tensor_output_topic': CLS_TENSOR_OUTPUT_TOPIC,
            }
        ],
        remappings=[
            ('tensor_pub', CLS_TENSOR_OUTPUT_TOPIC),
            ('tensor_sub', CLS_TENSOR_INPUT_TOPIC),
        ],
    )

    cls_decoder_node = ComposableNode(
        name='cls_decoder_node',
        package='isaac_ros_yolov26_cls',
        plugin='nvidia::isaac_ros::yolov26_cls::YoloV26ClsDecoderNode',
        parameters=[
            {
                'tensor_input_topic': CLS_TENSOR_INPUT_TOPIC,
                'confidence_threshold': float(cls_cfg['confidence_threshold']),
                'num_classes': int(cls_cfg['num_classes']),
                'class_topic': cls_class_topic,
            }
        ],
    )

    cls_container = ComposableNodeContainer(
        name='down_cam_cls_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            cls_image_format_converter,
            cls_tensor_rt_node,
            cls_decoder_node,
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    cls_encoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')
        ),
        launch_arguments={
            'input_image_width': str(cls_cfg['input_image_width']),
            'input_image_height': str(cls_cfg['input_image_height']),
            'network_image_width': str(cls_cfg['network_image_width']),
            'network_image_height': str(cls_cfg['network_image_height']),
            'image_mean': str(cls_cfg['image_mean']),
            'image_stddev': str(cls_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'down_cam_cls_container',
            'dnn_image_encoder_namespace': CLS_ENCODER_NAMESPACE,
            'image_input_topic': CLS_CONVERTED_IMAGE_TOPIC,
            'camera_info_input_topic': cls_cfg['camera_info_input_topic'],
            'tensor_output_topic': CLS_TENSOR_OUTPUT_TOPIC,
        }.items(),
    )

    cls_visualizer = (
        Node(
            package='isaac_ros_yolov26_cls',
            executable='isaac_ros_yolov26_cls_visualizer.py',
            name='cls_visualizer',
            parameters=[
                {
                    'class_topic': cls_class_topic,
                    'image_topic': f'/{CLS_ENCODER_NAMESPACE}/resize/image',
                    'output_image_topic': cls_visualized_topic,
                    'class_names_yaml': str(cls_cfg['class_names']),
                }
            ],
        )
        if bool(cls_cfg['enable_visualizer'])
        else None
    )

    # -------------------------------------------------------------------------
    # GStreamer — streams the segmentation visualization (bgr8)
    # -------------------------------------------------------------------------
    image_to_gstreamer_node = Node(
        package='image_to_gstreamer',
        executable='image_to_gstreamer_node',
        name='image_to_gstreamer_node',
        additional_env={'EGL_PLATFORM': 'surfaceless'},
        parameters=[
            {
                'input_topic': seg_visualized_topic,
                'host': '10.0.0.68',
                'port': 5001,
                'bitrate': 500000,
                'framerate': 15,
                'preset_level': 1,
                'iframe_interval': 15,
                'control_rate': 1,
                'pt': 96,
                'config_interval': 1,
                'format': 'BGR',
            }
        ],
        output='screen',
    )

    actions = [
        camera_container,
        seg_container,
        seg_encoder_launch,
        cls_container,
        cls_encoder_launch,
        image_to_gstreamer_node,
    ]

    if seg_visualizer:
        actions.append(seg_visualizer)
    if cls_visualizer:
        actions.append(cls_visualizer)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_camera',
                default_value='true',
                description='Enable FLIR Blackfly S camera component',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

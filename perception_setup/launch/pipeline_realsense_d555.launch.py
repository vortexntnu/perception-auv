"""RealSense D555 -> YOLO Segmentation -> GStreamer stream.

Pipeline:
1. RealSense D555 camera driver publishes raw RGB color image
2. image_undistort undistorts the raw color image
3. YOLO segmentation detects pipeline instances and produces a segmentation mask
   and visualization image
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# All pipeline-internal and output topics are prefixed to avoid collisions
# when running alongside other camera pipelines.
CAM_PREFIX = '/front_cam'

# Segmentation pipeline internal topics
SEG_CONVERTED_IMAGE_TOPIC = f'{CAM_PREFIX}/yolo_seg/internal/converted_image'
SEG_TENSOR_OUTPUT_TOPIC = f'{CAM_PREFIX}/yolo_seg/tensor_pub'
SEG_TENSOR_INPUT_TOPIC = f'{CAM_PREFIX}/yolo_seg/tensor_sub'
SEG_ENCODER_NAMESPACE = 'front_cam/yolo_seg_encoder/internal'


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')

    # Load camera config
    cameras_path = os.path.join(pkg_dir, 'config', 'cameras', 'cameras.yaml')
    with open(cameras_path) as f:
        cameras = yaml.safe_load(f)
    cam = cameras['realsense_d555']

    # Load segmentation model config
    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')) as f:
        seg_cfg = yaml.safe_load(f)

    models_dir = os.path.join(pkg_dir, 'models')

    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )

    # Prefix yaml-sourced topics
    seg_detection_topic = f'{CAM_PREFIX}{seg_cfg["detection_topic"]}'
    seg_mask_topic = f'{CAM_PREFIX}{seg_cfg["mask_topic"]}'
    seg_visualized_topic = f'{CAM_PREFIX}{seg_cfg["visualized_image_topic"]}'

    # -------------------------------------------------------------------------
    # Camera + Undistort
    # -------------------------------------------------------------------------
    camera_container = ComposableNodeContainer(
        name='front_cam_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera',
                namespace='camera',
                parameters=[
                    {
                        'enable_color': True,
                        'rgb_camera.color_profile': '896,504,15',
                        'rgb_camera.color_format': 'RGB8',
                        'rgb_camera.enable_auto_exposure': True,
                        'enable_depth': False,
                        'enable_infra1': False,
                        'enable_infra2': False,
                        'enable_gyro': False,
                        'enable_accel': False,
                        'enable_motion': False,
                        'publish_tf': False,
                        'enable_sync': False,
                    }
                ],
            ),
            ComposableNode(
                package='perception_setup',
                plugin='perception_setup::ImageUndistort',
                name='color_image_undistort',
                parameters=[
                    {
                        'image_topic': cam['raw_color_image_topic'],
                        'camera_info_file': calib_file,
                        'raw_camera_info_topic': cam['raw_color_camera_info_topic'],
                        'output_image_topic': cam['image_topic'],
                        'output_camera_info_topic': cam['camera_info_topic'],
                        'enable_undistort': LaunchConfiguration('enable_undistort'),
                        'image_qos': 'sensor_data',
                    }
                ],
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
        name='front_cam_seg_container',
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
            'component_container_name': 'front_cam_seg_container',
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
    # GStreamer — streams the segmentation visualization (rgb8)
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
                'port': 5000,
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
        image_to_gstreamer_node,
    ]

    if seg_visualizer:
        actions.append(seg_visualizer)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_undistort',
                default_value='true',
                description='Undistort color image before segmentation',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

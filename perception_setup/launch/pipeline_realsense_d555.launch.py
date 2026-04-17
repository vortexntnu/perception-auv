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


def _launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('perception_setup')
    models_dir = os.path.join(pkg_dir, 'models')
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')

    with open(os.path.join(pkg_dir, 'config', 'yolo', 'yolo_seg.yaml')) as f:
        seg_cfg = yaml.safe_load(f)

    calib_file = os.path.join(
        pkg_dir, 'config', 'cameras', 'color_realsense_d555_calib.yaml'
    )

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
                        'image_topic': '/camera/camera/color/image_raw',
                        'camera_info_file': calib_file,
                        'raw_camera_info_topic': '/camera/camera/color/camera_info',
                        'output_image_topic': '/realsense_d555/color/image_rect',
                        'output_camera_info_topic': '/realsense_d555/color/camera_info',
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
                'encoding_desired': 'rgb8',
                'image_width': 896,
                'image_height': 504,
                'input_qos': 'sensor_data',
            }
        ],
        remappings=[
            ('image_raw', '/realsense_d555/color/image_rect'),
            ('image', '/front_cam/yolo_seg/internal/converted_image'),
        ],
    )

    seg_tensor_rt_node = ComposableNode(
        name='seg_tensor_rt',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[
            {
                'model_file_path': os.path.join(models_dir, 'seg_pipe_real.onnx'),
                'engine_file_path': os.path.join(models_dir, 'seg_pipe_real.engine'),
                'output_binding_names': seg_cfg['output_binding_names'],
                'output_tensor_names': seg_cfg['output_tensor_names'],
                'input_tensor_names': seg_cfg['input_tensor_names'],
                'input_binding_names': seg_cfg['input_binding_names'],
                'verbose': True,
                'force_engine_update': False,
                'tensor_output_topic': '/front_cam/yolo_seg/tensor_pub',
            }
        ],
        remappings=[
            ('tensor_pub', '/front_cam/yolo_seg/tensor_pub'),
            ('tensor_sub', '/front_cam/yolo_seg/tensor_sub'),
        ],
    )

    seg_decoder_node = ComposableNode(
        name='seg_decoder_node',
        package='isaac_ros_yolov26_seg',
        plugin='nvidia::isaac_ros::yolov26_seg::YoloV26SegDecoderNode',
        parameters=[
            {
                'tensor_input_topic': '/front_cam/yolo_seg/tensor_sub',
                'confidence_threshold': float(seg_cfg['confidence_threshold']),
                'num_detections': int(seg_cfg['num_detections']),
                'mask_width': int(seg_cfg['mask_width']),
                'mask_height': int(seg_cfg['mask_height']),
                'num_protos': int(seg_cfg['num_protos']),
                'network_image_width': int(seg_cfg['network_image_width']),
                'network_image_height': int(seg_cfg['network_image_height']),
                'output_mask_width': 640,
                'output_mask_height': 640,
                'detections_topic': '/front_cam/seg_detections_output',
                'mask_topic': '/front_cam/yolo_seg_mask',
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
            'input_image_width': str(896),
            'input_image_height': str(504),
            'network_image_width': str(seg_cfg['network_image_width']),
            'network_image_height': str(seg_cfg['network_image_height']),
            'image_mean': str(seg_cfg['image_mean']),
            'image_stddev': str(seg_cfg['image_stddev']),
            'attach_to_shared_component_container': 'True',
            'component_container_name': 'front_cam_seg_container',
            'dnn_image_encoder_namespace': 'front_cam/yolo_seg_encoder/internal',
            'image_input_topic': '/front_cam/yolo_seg/internal/converted_image',
            'camera_info_input_topic': '/realsense_d555/color/camera_info',
            'tensor_output_topic': '/front_cam/yolo_seg/tensor_pub',
        }.items(),
    )

    seg_visualizer = Node(
        package='isaac_ros_yolov26_seg',
        executable='isaac_ros_yolov26_seg_visualizer.py',
        name='seg_visualizer',
        parameters=[
            {
                'detections_topic': '/front_cam/seg_detections_output',
                'image_topic': '/front_cam/yolo_seg_encoder/internal/resize/image',
                'mask_topic': '/front_cam/yolo_seg_mask',
                'output_image_topic': '/front_cam/yolo_seg_processed_image',
                'class_names_yaml': str({0: 'pipeline'}),
            }
        ],
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
                'input_topic': '/front_cam/yolo_seg_processed_image',
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
